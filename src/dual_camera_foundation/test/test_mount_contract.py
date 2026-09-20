from pathlib import Path

import numpy as np
import pytest

from dual_camera_foundation.hand_eye_calibration import (
    Calibration,
    Metrics,
    save_calibration,
)
from dual_camera_foundation.mount_tf_publisher import (
    validate_mount_calibration,
    validate_mount_contract,
)


def _metrics(
    sample_count=20,
    translation_rms_m=0.004,
    rotation_rms_deg=0.8,
    outlier_indices=(),
):
    return Metrics(
        sample_count=sample_count,
        mean_translation_m=(0.4, 0.1, 0.3),
        translation_std_m=(0.001, 0.001, 0.001),
        translation_rms_m=translation_rms_m,
        rotation_mean_deg=0.5,
        rotation_std_deg=0.2,
        rotation_rms_deg=rotation_rms_deg,
        translation_errors_m=(translation_rms_m,) * sample_count,
        rotation_errors_deg=(rotation_rms_deg,) * sample_count,
        outlier_indices=outlier_indices,
    )


def _mount_file(tmp_path: Path, metrics=None, child='wrist_d405_link'):
    path = tmp_path / 'd405_mount.yaml'
    transform = np.eye(4)
    transform[:3, 3] = [0.03, -0.02, 0.08]
    save_calibration(
        path,
        Calibration(
            transform,
            'tool0',
            child,
            'synthetic-test',
            metrics or _metrics(),
        ),
    )
    return path


def test_valid_mount_file_passes_launch_and_publisher_contract(tmp_path):
    path = _mount_file(tmp_path)
    calibration = validate_mount_calibration(path)
    assert calibration.camera_frame == 'wrist_d405_link'
    assert validate_mount_contract(
        True,
        True,
        path,
        'tool0',
        'wrist_d405_link',
        'wrist_d405_link',
    ) is not None


def test_d405_transform_requires_mount_calibration_enabled():
    with pytest.raises(ValueError, match='use_mount_calibration'):
        validate_mount_contract(
            True,
            False,
            '',
            'tool0',
            'wrist_d405_link',
            'wrist_d405_link',
        )


def test_mount_contract_rejects_missing_file_or_disconnected_child(tmp_path):
    with pytest.raises((FileNotFoundError, ValueError)):
        validate_mount_contract(
            True,
            True,
            tmp_path / 'missing.yaml',
            'tool0',
            'wrist_d405_link',
            'wrist_d405_link',
        )

    path = _mount_file(tmp_path)
    with pytest.raises(ValueError, match='mount child frame'):
        validate_mount_contract(
            True,
            True,
            path,
            'tool0',
            'other_camera_link',
            'wrist_d405_link',
        )


@pytest.mark.parametrize(
    'metrics, message',
    [
        (_metrics(sample_count=14), 'only 14 samples'),
        (_metrics(translation_rms_m=0.02), 'translation RMS'),
        (_metrics(rotation_rms_deg=3.0), 'rotation RMS'),
        (_metrics(outlier_indices=(4,)), 'outlier samples'),
    ],
)
def test_mount_contract_rejects_unvalidated_metrics(
    tmp_path,
    metrics,
    message,
):
    path = _mount_file(tmp_path, metrics)
    with pytest.raises(ValueError, match=message):
        validate_mount_calibration(path)
