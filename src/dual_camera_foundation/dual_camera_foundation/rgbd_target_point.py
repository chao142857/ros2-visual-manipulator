"""Configurable RGB-D pixel to ``PointStamped`` provider."""

from __future__ import annotations

from collections import deque
import math
import statistics
import struct

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def depth_at(message, u, v):
    if u < 0 or v < 0 or u >= message.width or v >= message.height:
        return None
    if message.encoding == '16UC1':
        offset = v * message.step + u * 2
        order = '>' if message.is_bigendian else '<'
        value = struct.unpack_from(order + 'H', message.data, offset)[0]
        return value / 1000.0 if value else None
    if message.encoding == '32FC1':
        offset = v * message.step + u * 4
        order = '>' if message.is_bigendian else '<'
        value = struct.unpack_from(order + 'f', message.data, offset)[0]
        return value if math.isfinite(value) and value > 0 else None
    raise ValueError(f'unsupported depth encoding: {message.encoding}')


def median_depth(message, u, v, radius=2):
    values = [
        depth_at(message, u + du, v + dv)
        for du in range(-radius, radius + 1)
        for dv in range(-radius, radius + 1)
    ]
    values = [value for value in values if value is not None]
    return statistics.median(values) if values else None


def intrinsics(message):
    """Use K when valid, otherwise fall back to P."""
    k = [float(value) for value in message.k]
    if all(math.isfinite(k[i]) for i in (0, 2, 4, 5)) and k[0] and k[4]:
        return k[0], k[4], k[2], k[5]
    p = [float(value) for value in message.p]
    if all(math.isfinite(p[i]) for i in (0, 2, 5, 6)) and p[0] and p[5]:
        return p[0], p[5], p[2], p[6]
    return None


def stamp_nanoseconds(stamp):
    """Return a positive ROS timestamp in nanoseconds, or ``None``."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return None
    value = sec * 1_000_000_000 + nanosec
    return value if value > 0 else None


def nearest_depth_frame(
    messages,
    pixel_stamp,
    max_depth_age_sec,
    future_tolerance_sec,
):
    """Select the closest eligible depth frame to an RGB/pixel timestamp.

    ``depth_stamp - pixel_stamp`` may be slightly negative (an older depth
    frame) or positive (a newer depth frame), but must remain inside the
    configured asymmetric bounds. Invalid and zero timestamps are ignored.
    """
    pixel_ns = stamp_nanoseconds(pixel_stamp)
    if pixel_ns is None:
        return None
    maximum_past_ns = int(float(max_depth_age_sec) * 1_000_000_000)
    maximum_future_ns = int(float(future_tolerance_sec) * 1_000_000_000)
    eligible = []
    for message in messages:
        depth_ns = stamp_nanoseconds(message.header.stamp)
        if depth_ns is None:
            continue
        delta_ns = depth_ns - pixel_ns
        if -maximum_past_ns <= delta_ns <= maximum_future_ns:
            eligible.append((abs(delta_ns), -depth_ns, message))
    if not eligible:
        return None
    # Prefer the closest frame; on an exact tie prefer the newer timestamp.
    return min(eligible, key=lambda item: item[:2])[2]


class RgbdTargetPoint(Node):
    """Consume pixel ``PointStamped`` messages and publish camera-frame XYZ."""

    def __init__(self):
        super().__init__('rgbd_target_point')
        defaults = {
            'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'pixel_topic': '/perception/target_pixel',
            'output_topic': '/perception/target_point_camera',
            'neighborhood_radius': 2,
            'min_depth_m': 0.05,
            'max_depth_m': 3.0,
            # At 30 Hz, 50 ms is already more than one frame interval. Keep
            # this tight because a moving wrist made the former 500 ms cache
            # window geometrically unsafe.
            'max_depth_age_sec': 0.05,
            'future_tolerance_sec': 0.05,
            'depth_queue_size': 8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        queue_size = int(self.get_parameter('depth_queue_size').value)
        if queue_size < 2 or queue_size > 120:
            raise ValueError('depth_queue_size must be between 2 and 120')
        self.depth_frames = deque(maxlen=queue_size)
        self.camera_intrinsics = None
        self.max_depth_age_sec = float(
            self.get_parameter('max_depth_age_sec').value
        )
        self.future_tolerance_sec = float(
            self.get_parameter('future_tolerance_sec').value
        )
        if (
            not math.isfinite(self.max_depth_age_sec)
            or self.max_depth_age_sec <= 0.0
            or self.max_depth_age_sec > 0.25
        ):
            raise ValueError(
                'max_depth_age_sec must be finite and between 0 and 0.25'
            )
        if (
            not math.isfinite(self.future_tolerance_sec)
            or self.future_tolerance_sec < 0.0
            or self.future_tolerance_sec > 0.25
        ):
            raise ValueError(
                'future_tolerance_sec must be finite and between 0 and 0.25'
            )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter('pixel_topic').value,
            self.pixel_callback,
            10,
        )
        self.publisher = self.create_publisher(
            PointStamped,
            self.get_parameter('output_topic').value,
            10,
        )

    def depth_callback(self, message):
        """Keep a bounded queue of stamped depth frames for nearest matching."""
        if (
            not str(message.header.frame_id).strip()
            or stamp_nanoseconds(message.header.stamp) is None
        ):
            return
        self.depth_frames.append(message)

    def info_callback(self, message):
        self.camera_intrinsics = intrinsics(message)

    def pixel_callback(self, message):
        if not self.depth_frames or self.camera_intrinsics is None:
            return
        depth_message = nearest_depth_frame(
            self.depth_frames,
            message.header.stamp,
            self.max_depth_age_sec,
            self.future_tolerance_sec,
        )
        if depth_message is None:
            return
        if not all(
            math.isfinite(float(value))
            for value in (message.point.x, message.point.y)
        ):
            return
        u, v = int(round(message.point.x)), int(round(message.point.y))
        radius = max(
            0,
            int(self.get_parameter('neighborhood_radius').value),
        )
        try:
            depth = median_depth(depth_message, u, v, radius)
        except (ValueError, struct.error, IndexError):
            return
        minimum = float(self.get_parameter('min_depth_m').value)
        maximum = float(self.get_parameter('max_depth_m').value)
        if depth is None or not minimum <= depth <= maximum:
            return
        fx, fy, cx, cy = self.camera_intrinsics
        output = PointStamped()
        # XYZ belongs to the selected depth frame, so TF must use its stamp.
        output.header = depth_message.header
        output.point.z = depth
        output.point.x = (u - cx) * depth / fx
        output.point.y = (v - cy) * depth / fy
        if all(
            math.isfinite(value)
            for value in (output.point.x, output.point.y, output.point.z)
        ):
            self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdTargetPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    'RgbdTargetPoint',
    'depth_at',
    'intrinsics',
    'main',
    'median_depth',
    'nearest_depth_frame',
    'stamp_nanoseconds',
]
