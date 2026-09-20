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

"""Pure validation and health-classification helpers for the wrist D405."""

from dataclasses import dataclass, field
import math
import re
from typing import Mapping, Optional, Sequence, Tuple


OK = 0
WARN = 1
ERROR = 2
STALE = 3

SERIAL_PATTERN = re.compile(r'^[0-9]+$')
VERSION_PATTERN = re.compile(r'^[0-9]+(?:\.[0-9]+)*$')


def decode_diagnostic_level(value):
    """Normalize ROS ``byte`` or integer diagnostic levels to an int."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        if len(value) != 1:
            raise ValueError('diagnostic level byte must contain one value')
        return int(value[0])
    level = int(value)
    if not 0 <= level <= 255:
        raise ValueError('diagnostic level must fit in one byte')
    return level


def encode_diagnostic_level(value):
    """Encode an integer for the ROS 2 Humble ``byte`` message field."""
    return bytes((decode_diagnostic_level(value),))


@dataclass(frozen=True)
class Evaluation:
    """Diagnostic classification independent of ROS message types."""

    level: int
    message: str
    values: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class ImageSnapshot:
    """Latest image metadata and recent host receive timestamps."""

    publishers: int = 0
    last_received_at: Optional[float] = None
    received_at: Tuple[float, ...] = ()
    width: int = 0
    height: int = 0
    encoding: str = ''
    frame_id: str = ''


@dataclass(frozen=True)
class CameraInfoSnapshot:
    """Latest camera-info metadata."""

    publishers: int = 0
    last_received_at: Optional[float] = None
    width: int = 0
    height: int = 0
    frame_id: str = ''


@dataclass(frozen=True)
class DeviceSnapshot:
    """Fields returned by the RealSense device_info service."""

    available: bool = False
    device_name: str = ''
    serial_number: str = ''
    firmware_version: str = ''
    usb_type_descriptor: str = ''
    firmware_update_id: str = ''
    physical_port: str = ''


@dataclass(frozen=True)
class DriverFrequencySnapshot:
    """Frequency evidence published by the upstream RealSense driver."""

    last_received_at: Optional[float] = None
    level: int = STALE
    message: str = ''
    actual_fps: Optional[float] = None
    target_fps: Optional[float] = None


@dataclass(frozen=True)
class SmokeSnapshot:
    """All evidence consumed by the bounded hardware smoke test."""

    now: float
    rgb: ImageSnapshot
    depth: ImageSnapshot
    color_info: CameraInfoSnapshot
    depth_info: CameraInfoSnapshot
    device: DeviceSnapshot
    driver_rgb: DriverFrequencySnapshot = field(
        default_factory=DriverFrequencySnapshot
    )
    driver_depth: DriverFrequencySnapshot = field(
        default_factory=DriverFrequencySnapshot
    )


def normalize_serial_no(value):
    """Return a plain numeric serial or fail fast for ambiguous input."""
    serial = str(value).strip()
    if serial in ('', "''", '""'):
        return ''
    if serial.startswith('_'):
        serial = serial[1:]
    if not serial or SERIAL_PATTERN.fullmatch(serial) is None:
        raise ValueError(
            'serial_no must be empty, decimal digits, or _ followed by digits'
        )
    return serial


def serial_for_realsense_launch(value):
    """Format a user-facing serial for the upstream ROS launch file."""
    serial = normalize_serial_no(value)
    return "''" if not serial else f'_{serial}'


def parse_version(value):
    """Convert a dotted numeric firmware version to a comparable tuple."""
    version = str(value).strip()
    if VERSION_PATTERN.fullmatch(version) is None:
        raise ValueError(f'invalid dotted numeric version: {value!r}')
    return tuple(int(component) for component in version.split('.'))


def version_at_least(actual, recommended):
    """Compare dotted versions after padding shorter tuples with zeros."""
    actual_parts = parse_version(actual)
    recommended_parts = parse_version(recommended)
    width = max(len(actual_parts), len(recommended_parts))
    actual_parts += (0,) * (width - len(actual_parts))
    recommended_parts += (0,) * (width - len(recommended_parts))
    return actual_parts >= recommended_parts


def estimate_fps(received_at: Sequence[float]):
    """Estimate receive FPS from monotonic timestamps."""
    if len(received_at) < 2:
        return 0.0
    elapsed = float(received_at[-1]) - float(received_at[0])
    if elapsed <= 0.0 or not math.isfinite(elapsed):
        return 0.0
    return (len(received_at) - 1) / elapsed


def evaluate_image(
    stream_name,
    snapshot,
    now,
    freshness_sec,
    expected_width,
    expected_height,
    expected_encodings,
    expected_fps,
    minimum_fps_ratio,
    low_fps_level=WARN,
):
    """Classify an image stream using graph, freshness, format, and FPS."""
    values = {
        'publishers': str(snapshot.publishers),
        'resolution': f'{snapshot.width}x{snapshot.height}',
        'encoding': snapshot.encoding or '<none>',
        'frame_id': snapshot.frame_id or '<none>',
    }
    if snapshot.publishers < 1:
        return Evaluation(ERROR, f'{stream_name} has no publisher', values)
    if snapshot.last_received_at is None:
        return Evaluation(ERROR, f'no {stream_name} image received', values)
    age = max(0.0, float(now) - snapshot.last_received_at)
    fps = estimate_fps(snapshot.received_at)
    values['age_sec'] = f'{age:.3f}'
    values['receive_fps'] = f'{fps:.2f}'
    if age > freshness_sec:
        return Evaluation(ERROR, f'{stream_name} image is stale', values)
    if (
        snapshot.width != expected_width
        or snapshot.height != expected_height
    ):
        return Evaluation(
            ERROR,
            f'{stream_name} resolution does not match configuration',
            values,
        )
    if snapshot.encoding.lower() not in {
        encoding.lower() for encoding in expected_encodings
    }:
        return Evaluation(
            ERROR,
            f'{stream_name} encoding is unsupported',
            values,
        )
    if not snapshot.frame_id:
        return Evaluation(ERROR, f'{stream_name} frame_id is empty', values)
    minimum_fps = expected_fps * minimum_fps_ratio
    if len(snapshot.received_at) >= 3 and fps < minimum_fps:
        values['minimum_fps'] = f'{minimum_fps:.2f}'
        return Evaluation(
            low_fps_level,
            f'{stream_name} receive FPS is low',
            values,
        )
    return Evaluation(OK, f'{stream_name} stream is healthy', values)


def evaluate_camera_info(
    stream_name,
    snapshot,
    now,
    freshness_sec,
    expected_width,
    expected_height,
):
    """Classify a CameraInfo stream."""
    values = {
        'publishers': str(snapshot.publishers),
        'resolution': f'{snapshot.width}x{snapshot.height}',
        'frame_id': snapshot.frame_id or '<none>',
    }
    if snapshot.publishers < 1:
        return Evaluation(
            ERROR,
            f'{stream_name} camera_info has no publisher',
            values,
        )
    if snapshot.last_received_at is None:
        return Evaluation(
            ERROR,
            f'no {stream_name} camera_info received',
            values,
        )
    age = max(0.0, float(now) - snapshot.last_received_at)
    values['age_sec'] = f'{age:.3f}'
    if age > freshness_sec:
        return Evaluation(
            ERROR,
            f'{stream_name} camera_info is stale',
            values,
        )
    if (
        snapshot.width != expected_width
        or snapshot.height != expected_height
    ):
        return Evaluation(
            ERROR,
            f'{stream_name} camera_info resolution mismatch',
            values,
        )
    if not snapshot.frame_id:
        return Evaluation(
            ERROR,
            f'{stream_name} camera_info frame_id is empty',
            values,
        )
    return Evaluation(OK, f'{stream_name} camera_info is healthy', values)


def evaluate_frame_consistency(image_name, image, camera_info):
    """Ensure an image and its calibration describe the same frame."""
    values = {
        'image_frame_id': image.frame_id or '<none>',
        'camera_info_frame_id': camera_info.frame_id or '<none>',
    }
    if not image.frame_id or not camera_info.frame_id:
        return Evaluation(
            ERROR,
            f'{image_name} frame consistency cannot be established',
            values,
        )
    if image.frame_id != camera_info.frame_id:
        return Evaluation(
            ERROR,
            f'{image_name} image and camera_info frame_id differ',
            values,
        )
    return Evaluation(
        OK,
        f'{image_name} image and calibration frames agree',
        values,
    )


def evaluate_driver_frequency(
    stream_name,
    snapshot,
    now,
    freshness_sec,
    expected_fps,
    minimum_fps_ratio,
):
    """Validate source frequency from upstream driver diagnostics."""
    values = {
        'actual_fps': (
            '<unknown>' if snapshot.actual_fps is None
            else f'{snapshot.actual_fps:.2f}'
        ),
        'target_fps': (
            '<unknown>' if snapshot.target_fps is None
            else f'{snapshot.target_fps:.2f}'
        ),
        'driver_message': snapshot.message or '<none>',
    }
    if snapshot.last_received_at is None:
        return Evaluation(
            ERROR,
            f'no {stream_name} driver frequency diagnostic received',
            values,
        )
    age = max(0.0, float(now) - snapshot.last_received_at)
    values['age_sec'] = f'{age:.3f}'
    if age > freshness_sec:
        return Evaluation(
            ERROR,
            f'{stream_name} driver frequency diagnostic is stale',
            values,
        )
    actual_fps = snapshot.actual_fps
    if actual_fps is None or not math.isfinite(actual_fps):
        return Evaluation(
            ERROR,
            f'{stream_name} driver frequency is unavailable',
            values,
        )
    minimum_fps = expected_fps * minimum_fps_ratio
    values['minimum_fps'] = f'{minimum_fps:.2f}'
    if actual_fps < minimum_fps or snapshot.level >= ERROR:
        return Evaluation(
            ERROR,
            f'{stream_name} source FPS is below the required minimum',
            values,
        )
    if snapshot.level == WARN:
        return Evaluation(
            WARN,
            f'{stream_name} source FPS passes the smoke threshold but '
            'the driver reports a warning',
            values,
        )
    return Evaluation(
        OK,
        f'{stream_name} source FPS is healthy',
        values,
    )


def evaluate_device(
    snapshot,
    expected_serial='',
    recommended_firmware='5.16.0.1',
    strict_firmware=False,
):
    """Classify model, serial binding, USB speed, and firmware."""
    expected_serial = normalize_serial_no(expected_serial)
    values = {
        'device_name': snapshot.device_name or '<unknown>',
        'serial_number': snapshot.serial_number or '<unknown>',
        'firmware_version': snapshot.firmware_version or '<unknown>',
        'usb_type_descriptor': snapshot.usb_type_descriptor or '<unknown>',
        'firmware_update_id': snapshot.firmware_update_id or '<unknown>',
        'physical_port': snapshot.physical_port or '<unknown>',
    }
    if not snapshot.available:
        return Evaluation(ERROR, 'device_info service is unavailable', values)
    if 'd405' not in snapshot.device_name.lower():
        return Evaluation(ERROR, 'connected device is not a D405', values)
    if expected_serial and snapshot.serial_number != expected_serial:
        values['expected_serial'] = expected_serial
        return Evaluation(ERROR, 'D405 serial number does not match', values)
    usb = snapshot.usb_type_descriptor.strip().lower()
    if not usb or usb.startswith('2') or 'usb2' in usb:
        return Evaluation(ERROR, 'D405 is not operating over USB 3', values)
    firmware_level = OK
    firmware_message = ''
    try:
        firmware_current = version_at_least(
            snapshot.firmware_version,
            recommended_firmware,
        )
    except ValueError:
        firmware_current = False
        firmware_message = 'D405 firmware version could not be parsed'
    if not firmware_current:
        firmware_level = ERROR if strict_firmware else WARN
        firmware_message = firmware_message or (
            'D405 firmware is older than the recommended version'
        )
        values['recommended_firmware'] = recommended_firmware
    if firmware_level != OK:
        return Evaluation(firmware_level, firmware_message, values)
    if not expected_serial:
        return Evaluation(
            WARN,
            'D405 is healthy but serial_no is unpinned',
            values,
        )
    return Evaluation(OK, 'D405 identity and link are healthy', values)


def evaluate_mount_tf(available, require_mount_tf, placeholder=False):
    """Classify the calibrated hand-eye mount transform."""
    if placeholder:
        level = ERROR if require_mount_tf else WARN
        return Evaluation(
            level,
            'identity mount TF is an uncalibrated placeholder',
            {'calibrated': 'false'},
        )
    if available:
        return Evaluation(
            OK,
            'tool0 to wrist_d405_link transform is available',
            {'calibrated': 'externally asserted'},
        )
    level = ERROR if require_mount_tf else WARN
    return Evaluation(
        level,
        'hand-eye mount is not calibrated; mount TF is missing',
        {'calibrated': 'false'},
    )


def evaluate_smoke(
    snapshot,
    expected_serial='',
    freshness_sec=1.0,
    expected_width=640,
    expected_height=480,
    expected_fps=30.0,
    minimum_fps_ratio=0.7,
    recommended_firmware='5.16.0.1',
    strict_firmware=False,
):
    """Return named evaluations for a bounded hardware smoke test."""
    evaluations = {
        'device': evaluate_device(
            snapshot.device,
            expected_serial,
            recommended_firmware,
            strict_firmware,
        ),
        'rgb': evaluate_image(
            'RGB',
            snapshot.rgb,
            snapshot.now,
            freshness_sec,
            expected_width,
            expected_height,
            ('rgb8', 'bgr8'),
            expected_fps,
            minimum_fps_ratio,
            WARN,
        ),
        'depth': evaluate_image(
            'depth',
            snapshot.depth,
            snapshot.now,
            freshness_sec,
            expected_width,
            expected_height,
            ('16UC1',),
            expected_fps,
            minimum_fps_ratio,
            WARN,
        ),
        'color_camera_info': evaluate_camera_info(
            'color',
            snapshot.color_info,
            snapshot.now,
            freshness_sec,
            expected_width,
            expected_height,
        ),
        'depth_camera_info': evaluate_camera_info(
            'depth',
            snapshot.depth_info,
            snapshot.now,
            freshness_sec,
            expected_width,
            expected_height,
        ),
    }
    evaluations['color_frame_consistency'] = evaluate_frame_consistency(
        'color', snapshot.rgb, snapshot.color_info
    )
    evaluations['depth_frame_consistency'] = evaluate_frame_consistency(
        'depth', snapshot.depth, snapshot.depth_info
    )
    evaluations['rgb_source_frequency'] = evaluate_driver_frequency(
        'RGB', snapshot.driver_rgb, snapshot.now,
        max(3.0, 3.0 * freshness_sec), expected_fps,
        minimum_fps_ratio,
    )
    evaluations['depth_source_frequency'] = evaluate_driver_frequency(
        'depth', snapshot.driver_depth, snapshot.now,
        max(3.0, 3.0 * freshness_sec), expected_fps,
        minimum_fps_ratio,
    )
    if len(snapshot.rgb.received_at) < 3:
        evaluations['rgb'] = Evaluation(
            ERROR,
            'insufficient RGB frames to validate FPS',
            evaluations['rgb'].values,
        )
    if len(snapshot.depth.received_at) < 3:
        evaluations['depth'] = Evaluation(
            ERROR,
            'insufficient depth frames to validate FPS',
            evaluations['depth'].values,
        )
    return evaluations
