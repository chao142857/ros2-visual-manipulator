from pathlib import Path

import numpy as np

from dual_camera_foundation.frame_contract import (
    tool_to_mount_from_optical,
)
from dual_camera_foundation.hand_eye_calibration import (
    Calibration,
    Sample,
    load_calibration,
    save_calibration,
    solve,
    validate,
)


def _rotation(axis, angle):
    vector = np.zeros(3)
    vector[axis] = 1.0
    skew = np.array([
        [0.0, -vector[2], vector[1]],
        [vector[2], 0.0, -vector[0]],
        [-vector[1], vector[0], 0.0],
    ])
    transform = np.eye(4)
    transform[:3, :3] = (
        np.eye(3)
        + np.sin(angle) * skew
        + (1.0 - np.cos(angle)) * (skew @ skew)
    )
    return transform


def _synthetic_samples():
    tool_camera = _rotation(2, 0.25) @ _rotation(1, -0.18)
    tool_camera[:3, 3] = [0.04, -0.02, 0.07]
    base_target = np.eye(4)
    base_target[:3, 3] = [0.5, 0.1, 0.3]
    samples = []
    for index in range(12):
        base_tool = (
            _rotation(2, 0.08 * index)
            @ _rotation(1, 0.04 * index * index)
            @ _rotation(0, -0.03 * index)
        )
        base_tool[:3, 3] = [
            0.2 + 0.01 * index,
            0.05 * np.sin(index),
            0.25 + 0.008 * index,
        ]
        camera_target = (
            np.linalg.inv(tool_camera)
            @ np.linalg.inv(base_tool)
            @ base_target
        )
        samples.append(Sample(base_tool, camera_target, str(index)))
    return tool_camera, samples


def test_opencv_and_numpy_backends_recover_known_transform():
    expected, samples = _synthetic_samples()
    for backend in ('opencv', 'numpy'):
        solved = solve(samples, backend=backend)
        assert np.allclose(solved, expected, atol=1.0e-8)
        metrics = validate(samples, solved)
        assert metrics.translation_rms_m < 1.0e-10
        assert metrics.rotation_rms_deg < 1.0e-5


def test_yaml_round_trip_converts_numpy_metrics(tmp_path: Path):
    expected, samples = _synthetic_samples()
    metrics = validate(samples, expected)
    output = tmp_path / 'optical_calibration.yaml'
    save_calibration(
        output,
        Calibration(
            expected,
            'tool0',
            'wrist_d405_color_optical_frame',
            'synthetic',
            metrics,
        ),
    )
    loaded = load_calibration(output)
    assert loaded.camera_frame == 'wrist_d405_color_optical_frame'
    assert np.allclose(loaded.T_tool_camera, expected)
    assert loaded.metrics.sample_count == len(samples)


def test_optical_solution_is_converted_to_mount_frame():
    tool_optical = np.eye(4)
    tool_optical[:3, 3] = [0.1, 0.2, 0.3]
    mount_optical = _rotation(0, 0.2)
    mount_optical[:3, 3] = [0.01, 0.02, 0.03]
    expected = tool_optical @ np.linalg.inv(mount_optical)
    assert np.allclose(
        tool_to_mount_from_optical(tool_optical, mount_optical),
        expected,
    )
