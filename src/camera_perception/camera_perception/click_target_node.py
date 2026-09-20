"""Convert a registered RGB-D image click into a stamped 3D point."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image

from camera_perception.camera_geometry import (
    intrinsics_from_camera_info,
    pixel_to_camera_point,
)
from camera_perception.depth_image import get_valid_depth_near_pixel


def validate_registered_streams(
    rgb_msg,
    depth_msg,
    camera_info_msg,
    expected_frame='',
):
    """Validate that RGB, depth and intrinsics share one pixel geometry."""
    if camera_info_msg is None:
        return False, 'no valid depth CameraInfo received'

    rgb_size = (int(rgb_msg.width), int(rgb_msg.height))
    depth_size = (int(depth_msg.width), int(depth_msg.height))
    info_size = (
        int(camera_info_msg.width),
        int(camera_info_msg.height),
    )

    if (
        min(*rgb_size, *depth_size, *info_size) <= 0
        or rgb_size != depth_size
        or depth_size != info_size
    ):
        return False, (
            f'size mismatch: RGB={rgb_size}, depth={depth_size}, '
            f'info={info_size}'
        )

    frames = (
        rgb_msg.header.frame_id.strip(),
        depth_msg.header.frame_id.strip(),
        camera_info_msg.header.frame_id.strip(),
    )
    if not all(frames) or len(set(frames)) != 1:
        return False, (
            f'frame mismatch: RGB={frames[0]!r}, depth={frames[1]!r}, '
            f'info={frames[2]!r}'
        )

    expected_frame = str(expected_frame).strip()
    if expected_frame and frames[0] != expected_frame:
        return False, (
            f'expected registered frame {expected_frame!r}, '
            f'got {frames[0]!r}'
        )

    return True, ''


class ClickTargetNode(Node):
    """Publish depth-backed 3D points from a deliberately selected pixel."""

    def __init__(self):
        super().__init__('click_target_node')

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/depth/camera_info',
        )
        self.declare_parameter('output_topic', '/camera/object_point')
        self.declare_parameter('depth_search_radius', 4)
        self.declare_parameter('enable_optical_flow_tracking', False)
        self.declare_parameter('expected_registered_frame', '')
        self.declare_parameter('warning_throttle_sec', 5.0)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).value
        self.output_topic = self.get_parameter('output_topic').value
        self.depth_search_radius = self.get_parameter(
            'depth_search_radius'
        ).value
        self.enable_optical_flow_tracking = self.get_parameter(
            'enable_optical_flow_tracking'
        ).value
        self.expected_registered_frame = self.get_parameter(
            'expected_registered_frame'
        ).value
        self.warning_throttle_sec = self.get_parameter(
            'warning_throttle_sec'
        ).value
        self._validate_parameters()

        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_msg = None
        self._intrinsics_signature = None

        self.stream_ready = False
        self.image_size = None
        self.clicked_pixel = None
        self.prev_gray = None
        self.track_point = None

        self.point_publisher = self.create_publisher(
            PointStamped,
            self.output_topic,
            10,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.rgb_subscription = Subscriber(self, Image, self.rgb_topic)
        self.depth_subscription = Subscriber(self, Image, self.depth_topic)
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_subscription, self.depth_subscription],
            queue_size=10,
            slop=0.05,
        )
        self.synchronizer.registerCallback(self.synced_callback)

        self.window_name = 'Astra Click Target'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        mode = (
            'OPTICAL FLOW (experimental)'
            if self.enable_optical_flow_tracking
            else 'FIXED PIXEL (grasp-safe default)'
        )
        self.get_logger().info(
            'Click Target Node started\n'
            f'  RGB      = {self.rgb_topic}\n'
            f'  Depth    = {self.depth_topic}\n'
            f'  Info     = {self.camera_info_topic}\n'
            f'  Output   = {self.output_topic}\n'
            f'  Mode     = {mode}\n'
            'Click the intended grasp point once.'
        )

    def _validate_parameters(self):
        if not all((
            self.rgb_topic,
            self.depth_topic,
            self.camera_info_topic,
            self.output_topic,
        )):
            raise ValueError('RGB-D topics and output_topic must be non-empty')
        if self.depth_search_radius < 0:
            raise ValueError('depth_search_radius must be non-negative')
        if (
            not np.isfinite(self.warning_throttle_sec)
            or self.warning_throttle_sec <= 0.0
        ):
            raise ValueError(
                'warning_throttle_sec must be finite and positive'
            )

    def _warn_throttled(self, message):
        self.get_logger().warning(
            message,
            throttle_duration_sec=self.warning_throttle_sec,
        )

    def _clear_selection(self):
        self.clicked_pixel = None
        self.track_point = None
        self.prev_gray = None

    def camera_info_callback(self, msg):
        intrinsics = intrinsics_from_camera_info(msg)
        if intrinsics is None:
            self.camera_info_msg = None
            self.fx = self.fy = self.cx = self.cy = None
            self.stream_ready = False
            self._clear_selection()
            self._warn_throttled(
                'Depth CameraInfo has no valid K/P intrinsics; '
                'target publication is disabled'
            )
            return

        fx, fy, cx, cy, source = intrinsics
        signature = (
            int(msg.width),
            int(msg.height),
            msg.header.frame_id.strip(),
            float(fx),
            float(fy),
            float(cx),
            float(cy),
            source,
        )
        self.camera_info_msg = msg
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy

        if signature != self._intrinsics_signature:
            self._intrinsics_signature = signature
            self.get_logger().info(
                'Camera intrinsics ready\n'
                f'  frame = {signature[2]}\n'
                f'  size = {signature[0]}x{signature[1]}\n'
                f'  source = {source}\n'
                f'  fx/fy = {fx:.3f}/{fy:.3f}\n'
                f'  cx/cy = {cx:.3f}/{cy:.3f}'
            )

    def publish_target_from_pixel(self, depth_msg, u, v):
        depth_m = get_valid_depth_near_pixel(
            depth_msg,
            u,
            v,
            radius=self.depth_search_radius,
        )
        if depth_m is None:
            self._warn_throttled(
                f'No valid depth near selected pixel ({u}, {v})'
            )
            return False

        point = pixel_to_camera_point(
            u,
            v,
            depth_m,
            self.fx,
            self.fy,
            self.cx,
            self.cy,
        )
        if point is None or not np.all(np.isfinite(point)):
            self._warn_throttled(
                f'Cannot deproject selected pixel ({u}, {v})'
            )
            return False

        point_msg = PointStamped()
        point_msg.header = depth_msg.header
        point_msg.point.x = float(point[0])
        point_msg.point.y = float(point[1])
        point_msg.point.z = float(point[2])
        self.point_publisher.publish(point_msg)
        return True

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8',
            )
        except Exception as error:
            self._warn_throttled(f'RGB conversion failed: {error}')
            return

        compatible, reason = validate_registered_streams(
            rgb_msg,
            depth_msg,
            self.camera_info_msg,
            self.expected_registered_frame,
        )
        if not compatible:
            self.stream_ready = False
            self.image_size = None
            self._clear_selection()
            self._warn_throttled(
                f'Refusing target publication: {reason}. '
                'Enable Astra depth_registration and use matching streams.'
            )
            return

        self.stream_ready = True
        self.image_size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if (
            self.enable_optical_flow_tracking
            and self.prev_gray is not None
            and self.track_point is not None
        ):
            next_point, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_gray,
                gray,
                self.track_point,
                None,
                winSize=(31, 31),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    20,
                    0.03,
                ),
            )
            tracking_ok = (
                next_point is not None
                and status is not None
                and status[0, 0] == 1
                and error is not None
                and error[0, 0] < 30.0
            )
            if tracking_ok:
                x, y = next_point[0, 0]
                u, v = int(round(x)), int(round(y))
                tracking_ok = (
                    0 <= u < frame.shape[1]
                    and 0 <= v < frame.shape[0]
                )
                if tracking_ok:
                    self.clicked_pixel = (u, v)
                    self.track_point = next_point

            if not tracking_ok:
                self._warn_throttled(
                    'Target tracking lost; click the object again'
                )
                self.clicked_pixel = None
                self.track_point = None

        if self.enable_optical_flow_tracking:
            self.prev_gray = gray
        else:
            self.prev_gray = None
            self.track_point = None

        if self.clicked_pixel is not None:
            u, v = self.clicked_pixel
            self.publish_target_from_pixel(depth_msg, u, v)
            cv2.drawMarker(
                frame,
                (u, v),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                24,
                2,
            )
            label = (
                'TRACKING'
                if self.enable_optical_flow_tracking
                else 'PIXEL LOCKED'
            )
            cv2.putText(
                frame,
                label,
                (u + 12, max(22, v - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        cv2.putText(
            frame,
            'Click target once | q = quit',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow(self.window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def mouse_callback(self, event, u, v, flags, param):
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if not self.stream_ready or self.image_size is None:
            self._warn_throttled(
                'No compatible registered RGB-D frame is ready'
            )
            return

        width, height = self.image_size
        if not (0 <= u < width and 0 <= v < height):
            self._warn_throttled(
                f'Click ({u}, {v}) is outside the RGB-D image'
            )
            return

        self.clicked_pixel = (int(u), int(v))
        if self.enable_optical_flow_tracking:
            if self.prev_gray is None:
                self.clicked_pixel = None
                self._warn_throttled('No RGB frame is ready for tracking')
                return
            self.track_point = np.array(
                [[[float(u), float(v)]]],
                dtype=np.float32,
            )
        else:
            self.track_point = None

        mode = (
            'tracking enabled'
            if self.enable_optical_flow_tracking
            else 'fixed pixel'
        )
        self.get_logger().info(
            f'Target locked at pixel ({u}, {v}); {mode}'
        )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClickTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
