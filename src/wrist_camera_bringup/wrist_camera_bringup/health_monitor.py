# Copyright 2026 chao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish standard ROS diagnostics for the wrist-mounted D405."""

from collections import deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from realsense2_camera_msgs.srv import DeviceInfo
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from wrist_camera_bringup.health import (
    CameraInfoSnapshot,
    DeviceSnapshot,
    ERROR,
    Evaluation,
    ImageSnapshot,
    OK,
    WARN,
    decode_diagnostic_level,
    encode_diagnostic_level,
    evaluate_camera_info,
    evaluate_device,
    evaluate_frame_consistency,
    evaluate_image,
    evaluate_mount_tf,
    normalize_serial_no,
)


def _absolute_camera_base(namespace, camera_name):
    """Join a namespace and camera name into one absolute ROS path."""
    namespace = str(namespace).strip('/')
    camera_name = str(camera_name).strip('/')
    if not namespace or not camera_name:
        raise ValueError('camera_namespace and camera_name must be non-empty')
    return f'/{namespace}/{camera_name}'


def _positive_finite(name, value):
    """Validate a positive finite float parameter."""
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f'{name} must be positive and finite')
    return number


class _ImageTracker:
    """Hold bounded metadata for one image stream."""

    def __init__(self):
        self.last_received_at = None
        self.received_at = deque(maxlen=240)
        self.width = 0
        self.height = 0
        self.encoding = ''
        self.frame_id = ''

    def record(self, message, received_at):
        """Record one image without retaining its large data buffer."""
        self.last_received_at = received_at
        self.received_at.append(received_at)
        self.width = int(message.width)
        self.height = int(message.height)
        self.encoding = str(message.encoding)
        self.frame_id = str(message.header.frame_id)

    def snapshot(self, publishers, now):
        """Build a pure snapshot using only the recent FPS window."""
        recent = tuple(
            stamp for stamp in self.received_at if now - stamp <= 3.0
        )
        return ImageSnapshot(
            publishers=publishers,
            last_received_at=self.last_received_at,
            received_at=recent,
            width=self.width,
            height=self.height,
            encoding=self.encoding,
            frame_id=self.frame_id,
        )


class _CameraInfoTracker:
    """Hold bounded metadata for one CameraInfo stream."""

    def __init__(self):
        self.last_received_at = None
        self.width = 0
        self.height = 0
        self.frame_id = ''

    def record(self, message, received_at):
        """Record one calibration message."""
        self.last_received_at = received_at
        self.width = int(message.width)
        self.height = int(message.height)
        self.frame_id = str(message.header.frame_id)

    def snapshot(self, publishers):
        """Build a pure camera-info snapshot."""
        return CameraInfoSnapshot(
            publishers=publishers,
            last_received_at=self.last_received_at,
            width=self.width,
            height=self.height,
            frame_id=self.frame_id,
        )


class WristD405HealthMonitor(Node):
    """Monitor D405 graph presence, stream health, identity, and mount TF."""

    def __init__(self):
        super().__init__('wrist_d405_health_monitor')
        self.declare_parameter('camera_namespace', 'wrist_camera')
        self.declare_parameter('camera_name', 'wrist_d405')
        self.declare_parameter('serial_no', '')
        self.declare_parameter('expected_width', 640)
        self.declare_parameter('expected_height', 480)
        self.declare_parameter('expected_fps', 30.0)
        self.declare_parameter('minimum_fps_ratio', 0.7)
        self.declare_parameter('freshness_sec', 1.0)
        self.declare_parameter('status_period_sec', 1.0)
        self.declare_parameter('startup_grace_sec', 5.0)
        self.declare_parameter('device_info_retry_sec', 5.0)
        self.declare_parameter('recommended_firmware', '5.16.0.1')
        self.declare_parameter('driver_diagnostics_topic', '/diagnostics')
        self.declare_parameter(
            'diagnostics_output_topic',
            '/wrist_camera/wrist_d405/health/diagnostics',
        )
        self.declare_parameter('mount_parent_frame', 'tool0')
        self.declare_parameter('mount_child_frame', 'wrist_d405_link')
        self.declare_parameter('require_mount_tf', False)
        self.declare_parameter('mount_tf_is_placeholder', False)
        self.declare_parameter('require_aligned_depth', False)

        camera_namespace = self.get_parameter('camera_namespace').value
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.camera_base = _absolute_camera_base(
            camera_namespace,
            self.camera_name,
        )
        self.expected_serial = normalize_serial_no(
            self.get_parameter('serial_no').value
        )
        self.expected_width = int(
            self.get_parameter('expected_width').value
        )
        self.expected_height = int(
            self.get_parameter('expected_height').value
        )
        if self.expected_width <= 0 or self.expected_height <= 0:
            raise ValueError('expected image dimensions must be positive')
        self.expected_fps = _positive_finite(
            'expected_fps', self.get_parameter('expected_fps').value
        )
        self.minimum_fps_ratio = float(
            self.get_parameter('minimum_fps_ratio').value
        )
        if not 0.0 < self.minimum_fps_ratio <= 1.0:
            raise ValueError('minimum_fps_ratio must be in (0, 1]')
        self.freshness_sec = _positive_finite(
            'freshness_sec', self.get_parameter('freshness_sec').value
        )
        self.status_period_sec = _positive_finite(
            'status_period_sec',
            self.get_parameter('status_period_sec').value,
        )
        self.startup_grace_sec = _positive_finite(
            'startup_grace_sec',
            self.get_parameter('startup_grace_sec').value,
        )
        self.device_info_retry_sec = _positive_finite(
            'device_info_retry_sec',
            self.get_parameter('device_info_retry_sec').value,
        )
        self.recommended_firmware = str(
            self.get_parameter('recommended_firmware').value
        )
        self.mount_parent_frame = str(
            self.get_parameter('mount_parent_frame').value
        )
        self.mount_child_frame = str(
            self.get_parameter('mount_child_frame').value
        )
        self.require_mount_tf = bool(
            self.get_parameter('require_mount_tf').value
        )
        self.mount_tf_is_placeholder = bool(
            self.get_parameter('mount_tf_is_placeholder').value
        )
        self.require_aligned_depth = bool(
            self.get_parameter('require_aligned_depth').value
        )

        self.topics = {
            'rgb': f'{self.camera_base}/color/image_rect_raw',
            'depth': f'{self.camera_base}/depth/image_rect_raw',
            'color_info': f'{self.camera_base}/color/camera_info',
            'depth_info': f'{self.camera_base}/depth/camera_info',
            'aligned_depth': (
                f'{self.camera_base}/aligned_depth_to_color/image_raw'
            ),
            'aligned_depth_info': (
                f'{self.camera_base}/aligned_depth_to_color/camera_info'
            ),
        }
        self._images = {
            'rgb': _ImageTracker(),
            'depth': _ImageTracker(),
            'aligned_depth': _ImageTracker(),
        }
        self._camera_infos = {
            'color_info': _CameraInfoTracker(),
            'depth_info': _CameraInfoTracker(),
            'aligned_depth_info': _CameraInfoTracker(),
        }
        self._started_at = time.monotonic()
        self._last_device_attempt_at = None
        self._device_future = None
        self._device = DeviceSnapshot()
        self._last_driver_diagnostic_at = None
        self._driver_level = OK
        self._driver_messages = []

        self.create_subscription(
            Image,
            self.topics['rgb'],
            lambda message: self._record_image('rgb', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.topics['depth'],
            lambda message: self._record_image('depth', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.topics['color_info'],
            lambda message: self._record_camera_info(
                'color_info', message
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.topics['depth_info'],
            lambda message: self._record_camera_info(
                'depth_info', message
            ),
            qos_profile_sensor_data,
        )
        if self.require_aligned_depth:
            self.create_subscription(
                Image,
                self.topics['aligned_depth'],
                lambda message: self._record_image(
                    'aligned_depth', message
                ),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                self.topics['aligned_depth_info'],
                lambda message: self._record_camera_info(
                    'aligned_depth_info', message
                ),
                qos_profile_sensor_data,
            )
        driver_diagnostics_topic = str(
            self.get_parameter('driver_diagnostics_topic').value
        )
        self.create_subscription(
            DiagnosticArray,
            driver_diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        diagnostics_output_topic = str(
            self.get_parameter('diagnostics_output_topic').value
        )
        if diagnostics_output_topic == driver_diagnostics_topic:
            raise ValueError(
                'diagnostics output must differ from driver input topic'
            )
        self._publisher = self.create_publisher(
            DiagnosticArray,
            diagnostics_output_topic,
            10,
        )
        self._device_client = self.create_client(
            DeviceInfo,
            f'{self.camera_base}/device_info',
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(self.status_period_sec, self._publish_health)
        self.get_logger().info(
            f'Monitoring wrist D405 at {self.camera_base}'
        )

    def _record_image(self, stream, message):
        self._images[stream].record(message, time.monotonic())

    def _record_camera_info(self, stream, message):
        self._camera_infos[stream].record(message, time.monotonic())

    def _diagnostic_matches_camera(self, status):
        text = ' '.join((status.name, status.hardware_id)).lower()
        candidates = [self.camera_name.lower(), 'd405']
        serial = self.expected_serial or self._device.serial_number
        if serial:
            candidates.append(serial.lower())
        return any(candidate and candidate in text for candidate in candidates)

    def _diagnostics_callback(self, message):
        matching = [
            status
            for status in message.status
            if self._diagnostic_matches_camera(status)
        ]
        if not matching:
            return
        self._last_driver_diagnostic_at = time.monotonic()
        self._driver_level = max(
            decode_diagnostic_level(status.level) for status in matching
        )
        self._driver_messages = [status.message for status in matching]

    def _request_device_info(self, now):
        if self._device_future is not None:
            return
        if (
            self._last_device_attempt_at is not None
            and now - self._last_device_attempt_at
            < self.device_info_retry_sec
        ):
            return
        if not self._device_client.service_is_ready():
            return
        self._last_device_attempt_at = now
        self._device_future = self._device_client.call_async(
            DeviceInfo.Request()
        )
        self._device_future.add_done_callback(self._device_info_done)

    def _device_info_done(self, future):
        self._device_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                f'D405 device_info request failed: {error}'
            )
            return
        self._device = DeviceSnapshot(
            available=True,
            device_name=str(response.device_name),
            serial_number=str(response.serial_number),
            firmware_version=str(response.firmware_version),
            usb_type_descriptor=str(response.usb_type_descriptor),
            firmware_update_id=str(response.firmware_update_id),
            physical_port=str(response.physical_port),
        )

    def _driver_evaluation(self, now):
        values = {
            'source_topic': str(
                self.get_parameter('driver_diagnostics_topic').value
            )
        }
        if self._last_driver_diagnostic_at is None:
            if now - self._started_at < self.startup_grace_sec:
                return Evaluation(WARN, 'waiting for driver diagnostics', values)
            return Evaluation(WARN, 'no matching driver diagnostics', values)
        age = now - self._last_driver_diagnostic_at
        values['age_sec'] = f'{age:.3f}'
        if age > max(3.0, 3.0 * self.status_period_sec):
            return Evaluation(ERROR, 'driver diagnostics are stale', values)
        if self._driver_level >= ERROR:
            return Evaluation(
                ERROR,
                '; '.join(self._driver_messages) or 'driver reports an error',
                values,
            )
        if self._driver_level == WARN:
            return Evaluation(
                WARN,
                '; '.join(self._driver_messages) or 'driver reports a warning',
                values,
            )
        return Evaluation(OK, 'driver diagnostics are healthy', values)

    @staticmethod
    def _to_status(name, evaluation, hardware_id):
        return DiagnosticStatus(
            level=encode_diagnostic_level(evaluation.level),
            name=name,
            message=evaluation.message,
            hardware_id=hardware_id,
            values=[
                KeyValue(key=str(key), value=str(value))
                for key, value in evaluation.values.items()
            ],
        )

    def _publish_health(self):
        now = time.monotonic()
        self._request_device_info(now)
        rgb = self._images['rgb'].snapshot(
            self.count_publishers(self.topics['rgb']), now
        )
        depth = self._images['depth'].snapshot(
            self.count_publishers(self.topics['depth']), now
        )
        color_info = self._camera_infos['color_info'].snapshot(
            self.count_publishers(self.topics['color_info'])
        )
        depth_info = self._camera_infos['depth_info'].snapshot(
            self.count_publishers(self.topics['depth_info'])
        )
        evaluations = {
            'rgb': evaluate_image(
                'RGB', rgb, now, self.freshness_sec,
                self.expected_width, self.expected_height,
                ('rgb8', 'bgr8'), self.expected_fps,
                self.minimum_fps_ratio,
            ),
            'depth': evaluate_image(
                'depth', depth, now, self.freshness_sec,
                self.expected_width, self.expected_height,
                ('16UC1',), self.expected_fps,
                self.minimum_fps_ratio,
            ),
            'color_camera_info': evaluate_camera_info(
                'color', color_info, now, self.freshness_sec,
                self.expected_width, self.expected_height,
            ),
            'depth_camera_info': evaluate_camera_info(
                'depth', depth_info, now, self.freshness_sec,
                self.expected_width, self.expected_height,
            ),
            'color_frame_consistency': evaluate_frame_consistency(
                'color', rgb, color_info
            ),
            'depth_frame_consistency': evaluate_frame_consistency(
                'depth', depth, depth_info
            ),
            'device': evaluate_device(
                self._device,
                self.expected_serial,
                self.recommended_firmware,
                False,
            ),
            'driver': self._driver_evaluation(now),
        }
        if self.require_aligned_depth:
            aligned_depth = self._images['aligned_depth'].snapshot(
                self.count_publishers(self.topics['aligned_depth']), now
            )
            aligned_info = self._camera_infos[
                'aligned_depth_info'
            ].snapshot(
                self.count_publishers(self.topics['aligned_depth_info'])
            )
            evaluations.update({
                'aligned_depth': evaluate_image(
                    'aligned depth-to-color', aligned_depth, now,
                    self.freshness_sec, self.expected_width,
                    self.expected_height, ('16UC1',), self.expected_fps,
                    self.minimum_fps_ratio,
                ),
                'aligned_depth_camera_info': evaluate_camera_info(
                    'aligned depth-to-color', aligned_info, now,
                    self.freshness_sec, self.expected_width,
                    self.expected_height,
                ),
                'aligned_depth_frame_consistency': (
                    evaluate_frame_consistency(
                        'aligned depth-to-color',
                        aligned_depth,
                        aligned_info,
                    )
                ),
                'aligned_depth_color_frame': evaluate_frame_consistency(
                    'aligned depth-to-color vs color calibration',
                    aligned_depth,
                    color_info,
                ),
            })
        mount_available = self._tf_buffer.can_transform(
            self.mount_parent_frame,
            self.mount_child_frame,
            Time(),
        )
        evaluations['mount_tf'] = evaluate_mount_tf(
            mount_available,
            self.require_mount_tf,
            self.mount_tf_is_placeholder,
        )
        worst_level = max(
            evaluation.level for evaluation in evaluations.values()
        )
        summary_message = {
            OK: 'wrist D405 is healthy',
            WARN: 'wrist D405 has warnings',
            ERROR: 'wrist D405 is unhealthy',
        }.get(worst_level, 'wrist D405 status is stale')
        evaluations = {
            'summary': Evaluation(worst_level, summary_message),
            **evaluations,
        }
        hardware_id = self._device.serial_number or (
            self.expected_serial or 'unidentified-d405'
        )
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.status = [
            self._to_status(
                f'{self.camera_base}/health/{name}',
                evaluation,
                hardware_id,
            )
            for name, evaluation in evaluations.items()
        ]
        self._publisher.publish(diagnostic_array)


def main(args=None):
    """Run the wrist D405 health monitor."""
    rclpy.init(args=args)
    node = None
    try:
        node = WristD405HealthMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == '__main__':
    main()
