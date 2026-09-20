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

"""Unit tests for pure D405 health classification."""

import pytest

from wrist_camera_bringup.health import (
    CameraInfoSnapshot,
    DeviceSnapshot,
    ERROR,
    ImageSnapshot,
    OK,
    WARN,
    decode_diagnostic_level,
    encode_diagnostic_level,
    estimate_fps,
    evaluate_camera_info,
    evaluate_device,
    evaluate_frame_consistency,
    evaluate_image,
    evaluate_mount_tf,
    parse_version,
    version_at_least,
)


@pytest.mark.parametrize('level', [0, 1, 2, 3])
def test_diagnostic_level_round_trips_humble_byte_field(level):
    encoded = encode_diagnostic_level(level)
    assert encoded == bytes((level,))
    assert decode_diagnostic_level(encoded) == level
    assert decode_diagnostic_level(level) == level


def test_invalid_diagnostic_level_bytes_are_rejected():
    with pytest.raises(ValueError, match='one value'):
        decode_diagnostic_level(b'')
    with pytest.raises(ValueError, match='fit in one byte'):
        encode_diagnostic_level(256)


def healthy_image(**changes):
    values = {
        'publishers': 1,
        'last_received_at': 10.0,
        'received_at': (9.90, 9.94, 9.98, 10.0),
        'width': 640,
        'height': 480,
        'encoding': 'rgb8',
        'frame_id': 'wrist_d405_color_optical_frame',
    }
    values.update(changes)
    return ImageSnapshot(**values)


def healthy_info(**changes):
    values = {
        'publishers': 1,
        'last_received_at': 10.0,
        'width': 640,
        'height': 480,
        'frame_id': 'wrist_d405_color_optical_frame',
    }
    values.update(changes)
    return CameraInfoSnapshot(**values)


def healthy_device(**changes):
    values = {
        'available': True,
        'device_name': 'Intel RealSense D405',
        'serial_number': '1',
        'firmware_version': '5.16.0.1',
        'usb_type_descriptor': '3.2',
        'firmware_update_id': '413...',
        'physical_port': '4-2',
    }
    values.update(changes)
    return DeviceSnapshot(**values)


def classify_image(snapshot, low_fps_level=WARN):
    return evaluate_image(
        'RGB',
        snapshot,
        now=10.1,
        freshness_sec=1.0,
        expected_width=640,
        expected_height=480,
        expected_encodings=('rgb8', 'bgr8'),
        expected_fps=30.0,
        minimum_fps_ratio=0.7,
        low_fps_level=low_fps_level,
    )


def test_fps_estimation_handles_valid_and_degenerate_samples():
    assert estimate_fps((0.0, 0.5, 1.0)) == pytest.approx(2.0)
    assert estimate_fps(()) == 0.0
    assert estimate_fps((1.0, 1.0)) == 0.0


def test_healthy_image_and_camera_info_pass():
    assert classify_image(healthy_image()).level == OK
    result = evaluate_camera_info(
        'color', healthy_info(), 10.1, 1.0, 640, 480
    )
    assert result.level == OK


@pytest.mark.parametrize(
    ('snapshot', 'message'),
    [
        (healthy_image(publishers=0), 'no publisher'),
        (healthy_image(last_received_at=None), 'no RGB image'),
        (healthy_image(last_received_at=8.0), 'stale'),
        (healthy_image(width=848), 'resolution'),
        (healthy_image(encoding='mono8'), 'encoding'),
        (healthy_image(frame_id=''), 'frame_id'),
    ],
)
def test_image_failures_are_errors(snapshot, message):
    result = classify_image(snapshot)
    assert result.level == ERROR
    assert message in result.message


def test_low_fps_can_be_warning_for_health_or_error_for_smoke():
    slow = healthy_image(received_at=(8.0, 9.0, 10.0))
    assert classify_image(slow).level == WARN
    assert classify_image(slow, low_fps_level=ERROR).level == ERROR


def test_camera_info_and_frame_consistency_failures_are_detected():
    stale = healthy_info(last_received_at=8.0)
    assert evaluate_camera_info(
        'color', stale, 10.1, 1.0, 640, 480
    ).level == ERROR
    mismatch = healthy_info(frame_id='different_optical_frame')
    assert evaluate_frame_consistency(
        'color', healthy_image(), mismatch
    ).level == ERROR
    assert evaluate_frame_consistency(
        'color', healthy_image(), healthy_info()
    ).level == OK


def test_pinned_device_identity_and_usb3_pass():
    result = evaluate_device(
        healthy_device(), expected_serial='1'
    )
    assert result.level == OK


def test_unpinned_device_is_a_warning():
    result = evaluate_device(healthy_device(), expected_serial='')
    assert result.level == WARN
    assert 'unpinned' in result.message


@pytest.mark.parametrize(
    ('device', 'expected_serial', 'message'),
    [
        (healthy_device(available=False), '', 'unavailable'),
        (healthy_device(device_name='Intel RealSense D455'), '', 'not a D405'),
        (healthy_device(serial_number='0'), '1', 'does not match'),
        (healthy_device(usb_type_descriptor='2.1'), '', 'not operating over USB 3'),
        (healthy_device(usb_type_descriptor='USB2'), '', 'not operating over USB 3'),
        (healthy_device(usb_type_descriptor=''), '', 'not operating over USB 3'),
    ],
)
def test_device_identity_and_link_failures(device, expected_serial, message):
    result = evaluate_device(device, expected_serial=expected_serial)
    assert result.level == ERROR
    assert message in result.message


def test_old_firmware_warns_or_fails_in_strict_mode():
    old = healthy_device(firmware_version='5.15.0.2')
    warning = evaluate_device(old, expected_serial='1')
    failure = evaluate_device(
        old,
        expected_serial='1',
        strict_firmware=True,
    )
    assert warning.level == WARN
    assert failure.level == ERROR


def test_version_comparison_is_numeric_and_padded():
    assert parse_version('5.16.0.1') == (5, 16, 0, 1)
    assert version_at_least('5.16.1', '5.16.0.1')
    assert version_at_least('5.16', '5.16.0.0')
    assert not version_at_least('5.9.9', '5.16.0.1')
    with pytest.raises(ValueError):
        parse_version('5.17-beta')


def test_mount_tf_policy_distinguishes_missing_and_placeholder():
    assert evaluate_mount_tf(True, False).level == OK
    assert evaluate_mount_tf(False, False).level == WARN
    assert evaluate_mount_tf(False, True).level == ERROR
    placeholder = evaluate_mount_tf(True, False, placeholder=True)
    assert placeholder.level == WARN
    assert placeholder.values['calibrated'] == 'false'
