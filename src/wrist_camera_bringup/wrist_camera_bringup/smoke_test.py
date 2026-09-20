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

"""Bounded command-line smoke test for a running wrist D405 driver."""

from collections import deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from realsense2_camera_msgs.srv import DeviceInfo
from sensor_msgs.msg import CameraInfo, Image

from wrist_camera_bringup.health import (
    CameraInfoSnapshot,
    DeviceSnapshot,
    DriverFrequencySnapshot,
    ERROR,
    ImageSnapshot,
    SmokeSnapshot,
    evaluate_smoke,
    decode_diagnostic_level,
    normalize_serial_no,
)


def _camera_base(namespace, camera_name):
    namespace = str(namespace).strip('/')
    camera_name = str(camera_name).strip('/')
    if not namespace or not camera_name:
        raise ValueError('camera_namespace and camera_name must be non-empty')
    return f'/{namespace}/{camera_name}'


class WristD405SmokeTest(Node):
    """Collect just enough evidence to validate a real D405 deployment."""

    def __init__(self):
        super().__init__('wrist_d405_smoke_test')
        self.declare_parameter('camera_namespace', 'wrist_camera')
        self.declare_parameter('camera_name', 'wrist_d405')
        self.declare_parameter(
            'expected_serial',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('duration_sec', 8.0)
        self.declare_parameter('freshness_sec', 1.0)
        self.declare_parameter('expected_width', 640)
        self.declare_parameter('expected_height', 480)
        self.declare_parameter('expected_fps', 30.0)
        self.declare_parameter('minimum_fps_ratio', 0.7)
        self.declare_parameter('recommended_firmware', '5.16.0.1')
        self.declare_parameter('strict_firmware', False)
        self.declare_parameter('driver_diagnostics_topic', '/diagnostics')

        self.camera_base = _camera_base(
            self.get_parameter('camera_namespace').value,
            self.get_parameter('camera_name').value,
        )
        self.expected_serial = normalize_serial_no(
            self.get_parameter('expected_serial').value
        )
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.freshness_sec = float(
            self.get_parameter('freshness_sec').value
        )
        self.expected_width = int(
            self.get_parameter('expected_width').value
        )
        self.expected_height = int(
            self.get_parameter('expected_height').value
        )
        self.expected_fps = float(
            self.get_parameter('expected_fps').value
        )
        self.minimum_fps_ratio = float(
            self.get_parameter('minimum_fps_ratio').value
        )
        self.recommended_firmware = str(
            self.get_parameter('recommended_firmware').value
        )
        self.strict_firmware = bool(
            self.get_parameter('strict_firmware').value
        )
        finite_positive = (
            self.duration_sec,
            self.freshness_sec,
            self.expected_fps,
        )
        if any(not math.isfinite(value) or value <= 0.0
               for value in finite_positive):
            raise ValueError('duration, freshness, and FPS must be positive')
        if self.expected_width <= 0 or self.expected_height <= 0:
            raise ValueError('expected dimensions must be positive')
        if not 0.0 < self.minimum_fps_ratio <= 1.0:
            raise ValueError('minimum_fps_ratio must be in (0, 1]')

        self.topics = {
            'rgb': f'{self.camera_base}/color/image_rect_raw',
            'depth': f'{self.camera_base}/depth/image_rect_raw',
            'color_info': f'{self.camera_base}/color/camera_info',
            'depth_info': f'{self.camera_base}/depth/camera_info',
        }
        self._images = {
            'rgb': ImageSnapshot(),
            'depth': ImageSnapshot(),
        }
        self._image_stamps = {
            'rgb': deque(maxlen=600),
            'depth': deque(maxlen=600),
        }
        self._infos = {
            'color_info': CameraInfoSnapshot(),
            'depth_info': CameraInfoSnapshot(),
        }
        self.device = DeviceSnapshot()
        self._driver_frequencies = {
            'rgb': DriverFrequencySnapshot(),
            'depth': DriverFrequencySnapshot(),
        }
        self._device_future = None
        self._image_callbacks = MutuallyExclusiveCallbackGroup()
        self._metadata_callbacks = MutuallyExclusiveCallbackGroup()
        self._device_callbacks = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Image,
            self.topics['rgb'],
            lambda message: self._image_callback('rgb', message),
            qos_profile_sensor_data,
            callback_group=self._image_callbacks,
        )
        self.create_subscription(
            Image,
            self.topics['depth'],
            lambda message: self._image_callback('depth', message),
            qos_profile_sensor_data,
            callback_group=self._image_callbacks,
        )
        self.create_subscription(
            CameraInfo,
            self.topics['color_info'],
            lambda message: self._info_callback('color_info', message),
            qos_profile_sensor_data,
            callback_group=self._metadata_callbacks,
        )
        self.create_subscription(
            CameraInfo,
            self.topics['depth_info'],
            lambda message: self._info_callback('depth_info', message),
            qos_profile_sensor_data,
            callback_group=self._metadata_callbacks,
        )
        self._device_client = self.create_client(
            DeviceInfo,
            f'{self.camera_base}/device_info',
            callback_group=self._device_callbacks,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter('driver_diagnostics_topic').value),
            self._diagnostics_callback,
            10,
            callback_group=self._metadata_callbacks,
        )
        self.create_timer(
            0.2,
            self._request_device_info,
            callback_group=self._device_callbacks,
        )

    def _diagnostics_callback(self, message):
        """Record upstream source frequency without retaining diagnostics."""
        now = time.monotonic()
        for status in message.status:
            name = str(status.name).lower()
            if self.expected_serial:
                if str(status.hardware_id) != self.expected_serial:
                    continue
            elif 'wrist_d405' not in name:
                continue
            stream = None
            if name.endswith(': color'):
                stream = 'rgb'
            elif name.endswith(': depth'):
                stream = 'depth'
            if stream is None:
                continue
            values = {
                str(item.key): str(item.value) for item in status.values
            }
            try:
                actual_fps = float(values['Actual frequency (Hz)'])
                target_fps = float(values['Target frequency (Hz)'])
            except (KeyError, ValueError):
                actual_fps = None
                target_fps = None
            self._driver_frequencies[stream] = DriverFrequencySnapshot(
                last_received_at=now,
                level=decode_diagnostic_level(status.level),
                message=str(status.message),
                actual_fps=actual_fps,
                target_fps=target_fps,
            )

    def _image_callback(self, stream, message):
        now = time.monotonic()
        self._image_stamps[stream].append(now)
        self._images[stream] = ImageSnapshot(
            last_received_at=now,
            received_at=tuple(self._image_stamps[stream]),
            width=int(message.width),
            height=int(message.height),
            encoding=str(message.encoding),
            frame_id=str(message.header.frame_id),
        )

    def _info_callback(self, stream, message):
        self._infos[stream] = CameraInfoSnapshot(
            last_received_at=time.monotonic(),
            width=int(message.width),
            height=int(message.height),
            frame_id=str(message.header.frame_id),
        )

    def _request_device_info(self):
        if self.device.available or self._device_future is not None:
            return
        if not self._device_client.service_is_ready():
            return
        self._device_future = self._device_client.call_async(
            DeviceInfo.Request()
        )
        self._device_future.add_done_callback(self._device_info_done)

    def _device_info_done(self, future):
        self._device_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f'device_info failed: {error}')
            return
        self.device = DeviceSnapshot(
            available=True,
            device_name=str(response.device_name),
            serial_number=str(response.serial_number),
            firmware_version=str(response.firmware_version),
            usb_type_descriptor=str(response.usb_type_descriptor),
            firmware_update_id=str(response.firmware_update_id),
            physical_port=str(response.physical_port),
        )

    def snapshot(self):
        """Return a pure snapshot with current graph publisher counts."""
        now = time.monotonic()
        images = {}
        for stream in ('rgb', 'depth'):
            previous = self._images[stream]
            recent = tuple(
                stamp
                for stamp in self._image_stamps[stream]
                if now - stamp <= 3.0
            )
            images[stream] = ImageSnapshot(
                publishers=self.count_publishers(self.topics[stream]),
                last_received_at=previous.last_received_at,
                received_at=recent,
                width=previous.width,
                height=previous.height,
                encoding=previous.encoding,
                frame_id=previous.frame_id,
            )
        infos = {}
        for stream in ('color_info', 'depth_info'):
            previous = self._infos[stream]
            infos[stream] = CameraInfoSnapshot(
                publishers=self.count_publishers(self.topics[stream]),
                last_received_at=previous.last_received_at,
                width=previous.width,
                height=previous.height,
                frame_id=previous.frame_id,
            )
        return SmokeSnapshot(
            now=now,
            rgb=images['rgb'],
            depth=images['depth'],
            color_info=infos['color_info'],
            depth_info=infos['depth_info'],
            device=self.device,
            driver_rgb=self._driver_frequencies['rgb'],
            driver_depth=self._driver_frequencies['depth'],
        )

    def evaluate(self):
        """Evaluate all mandatory smoke-test checks."""
        return evaluate_smoke(
            self.snapshot(),
            expected_serial=self.expected_serial,
            freshness_sec=self.freshness_sec,
            expected_width=self.expected_width,
            expected_height=self.expected_height,
            expected_fps=self.expected_fps,
            minimum_fps_ratio=self.minimum_fps_ratio,
            recommended_firmware=self.recommended_firmware,
            strict_firmware=self.strict_firmware,
        )


def main(args=None):
    """Run for a bounded duration and exit nonzero on any failed check."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = WristD405SmokeTest()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        deadline = time.monotonic() + node.duration_sec
        node.get_logger().info(
            f'Collecting D405 evidence for {node.duration_sec:.1f}s'
        )
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        evaluations = node.evaluate()
        failed = False
        labels = {0: 'PASS', 1: 'WARN', 2: 'FAIL', 3: 'STALE'}
        for name, evaluation in evaluations.items():
            label = labels.get(evaluation.level, 'UNKNOWN')
            line = f'[{label}] {name}: {evaluation.message}'
            if evaluation.values:
                details = ', '.join(
                    f'{key}={value}'
                    for key, value in evaluation.values.items()
                )
                line = f'{line} ({details})'
            if evaluation.level >= ERROR:
                failed = True
                node.get_logger().error(line)
            elif evaluation.level == 1:
                node.get_logger().warning(line)
            else:
                node.get_logger().info(line)
        if failed:
            node.get_logger().error('WRIST_D405_SMOKE_TEST_FAILED')
            return 1
        node.get_logger().info('WRIST_D405_SMOKE_TEST_PASSED')
        return 0
    except Exception as error:
        if node is not None:
            node.get_logger().error(f'smoke test could not start: {error}')
        else:
            print(f'wrist D405 smoke test could not start: {error}')
        return 2
    finally:
        if node is not None:
            try:
                if executor is not None:
                    executor.remove_node(node)
                    executor.shutdown(timeout_sec=1.0)
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == '__main__':
    raise SystemExit(main())
