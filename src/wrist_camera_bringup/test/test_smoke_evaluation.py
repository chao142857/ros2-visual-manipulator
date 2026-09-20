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

"""Mocked smoke-test evaluation without ROS or physical hardware."""

from wrist_camera_bringup.health import (
    CameraInfoSnapshot,
    DeviceSnapshot,
    DriverFrequencySnapshot,
    ERROR,
    ImageSnapshot,
    OK,
    SmokeSnapshot,
    WARN,
    evaluate_smoke,
)


def make_snapshot(
    rgb_stamps=(9.90, 9.94, 9.98, 10.02, 10.06),
    depth_stamps=(9.90, 9.94, 9.98, 10.02, 10.06),
):
    rgb = ImageSnapshot(
        publishers=1,
        last_received_at=10.06,
        received_at=rgb_stamps,
        width=640,
        height=480,
        encoding='rgb8',
        frame_id='wrist_d405_color_optical_frame',
    )
    depth = ImageSnapshot(
        publishers=1,
        last_received_at=10.06,
        received_at=depth_stamps,
        width=640,
        height=480,
        encoding='16UC1',
        frame_id='wrist_d405_depth_optical_frame',
    )
    return SmokeSnapshot(
        now=10.1,
        rgb=rgb,
        depth=depth,
        color_info=CameraInfoSnapshot(
            publishers=1,
            last_received_at=10.05,
            width=640,
            height=480,
            frame_id='wrist_d405_color_optical_frame',
        ),
        depth_info=CameraInfoSnapshot(
            publishers=1,
            last_received_at=10.05,
            width=640,
            height=480,
            frame_id='wrist_d405_depth_optical_frame',
        ),
        device=DeviceSnapshot(
            available=True,
            device_name='Intel RealSense D405',
            serial_number='1',
            firmware_version='5.16.0.1',
            usb_type_descriptor='3.2',
            firmware_update_id='413...',
            physical_port='4-2',
        ),
        driver_rgb=DriverFrequencySnapshot(
            last_received_at=10.0,
            level=OK,
            message='Desired frequency met',
            actual_fps=29.0,
            target_fps=30.0,
        ),
        driver_depth=DriverFrequencySnapshot(
            last_received_at=10.0,
            level=OK,
            message='Desired frequency met',
            actual_fps=29.5,
            target_fps=30.0,
        ),
    )


def test_complete_pinned_smoke_snapshot_passes():
    results = evaluate_smoke(
        make_snapshot(), expected_serial='1'
    )
    assert results
    assert all(result.level == OK for result in results.values())


def test_complete_unpinned_snapshot_warns_only_for_identity():
    results = evaluate_smoke(make_snapshot(), expected_serial='')
    assert results['device'].level == WARN
    assert all(
        result.level == OK
        for name, result in results.items()
        if name != 'device'
    )


def test_smoke_requires_enough_frames_to_measure_fps():
    results = evaluate_smoke(
        make_snapshot(rgb_stamps=(10.0, 10.05)),
        expected_serial='1',
    )
    assert results['rgb'].level == ERROR
    assert 'insufficient' in results['rgb'].message


def test_smoke_warns_for_slow_python_receive_but_uses_source_rate():
    slow = (7.0, 8.0, 9.0, 10.0)
    results = evaluate_smoke(
        make_snapshot(rgb_stamps=slow, depth_stamps=slow),
        expected_serial='1',
    )
    assert results['rgb'].level == WARN
    assert results['depth'].level == WARN
    assert 'FPS is low' in results['rgb'].message
    assert results['rgb_source_frequency'].level == OK
    assert results['depth_source_frequency'].level == OK


def test_smoke_fails_when_upstream_source_rate_is_low():
    snapshot = make_snapshot()
    snapshot = SmokeSnapshot(
        now=snapshot.now,
        rgb=snapshot.rgb,
        depth=snapshot.depth,
        color_info=snapshot.color_info,
        depth_info=snapshot.depth_info,
        device=snapshot.device,
        driver_rgb=DriverFrequencySnapshot(
            last_received_at=10.0,
            level=WARN,
            message='Frequency too low.',
            actual_fps=15.0,
            target_fps=30.0,
        ),
        driver_depth=snapshot.driver_depth,
    )
    results = evaluate_smoke(
        snapshot, expected_serial='1'
    )
    assert results['rgb_source_frequency'].level == ERROR
    assert 'below' in results['rgb_source_frequency'].message


def test_smoke_detects_usb2_wrong_serial_and_missing_stream():
    snapshot = make_snapshot()
    snapshot = SmokeSnapshot(
        now=snapshot.now,
        rgb=ImageSnapshot(),
        depth=snapshot.depth,
        color_info=snapshot.color_info,
        depth_info=snapshot.depth_info,
        device=DeviceSnapshot(
            available=True,
            device_name='Intel RealSense D405',
            serial_number='0',
            firmware_version='5.16.0.1',
            usb_type_descriptor='2.1',
        ),
        driver_rgb=snapshot.driver_rgb,
        driver_depth=snapshot.driver_depth,
    )
    results = evaluate_smoke(
        snapshot, expected_serial='1'
    )
    assert results['device'].level == ERROR
    assert results['rgb'].level == ERROR
