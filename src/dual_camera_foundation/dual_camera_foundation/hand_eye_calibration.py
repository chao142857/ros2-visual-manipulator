            
"""Validated eye-in-hand calibration with explicit transform directions.

The convention is ``T_A_B``: a point in frame B is mapped into frame A.
Each sample contains ``T_base_tool`` and ``T_camera_target``.  The stationary
target constraint is::

    T_base_target = T_base_tool @ T_tool_camera @ T_camera_target

The returned transform is therefore ``T_tool_camera``.  This module has no
ROS dependency so it can be tested on the development machine before moving
the workspace to Ubuntu.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping, Optional, Sequence

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


SCHEMA_SAMPLES = 'dual_camera_foundation.hand_eye_samples'
SCHEMA_CALIBRATION = 'dual_camera_foundation.hand_eye_calibration'
SCHEMA_VERSION = 1
TRANSFORM_CONVENTION = 'T_A_B maps coordinates from frame B into frame A'
METHODS = ('TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS')


def as_transform(value: Any, name: str = 'transform') -> np.ndarray:
    """Validate and copy a finite rigid homogeneous transform."""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f'{name} must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError(f'{name} has an invalid homogeneous row')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5):
        raise ValueError(f'{name} rotation is not orthonormal')
    if np.linalg.det(rotation) <= 0:
        raise ValueError(f'{name} rotation determinant must be positive')
    return matrix.copy()


def inverse(transform: Any) -> np.ndarray:
    """Invert a rigid transform."""
    matrix = as_transform(transform)
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def _project_rotation(rotation: Any) -> np.ndarray:
    u, _, vh = np.linalg.svd(np.asarray(rotation, dtype=float))
    result = u @ vh
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vh
    return result


def _make_transform(rotation: Any, translation: Any) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _project_rotation(rotation)
    vector = np.asarray(translation, dtype=float).reshape(-1)
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise ValueError('translation must contain three finite values')
    result[:3, 3] = vector
    return result


def rotation_angle_deg(rotation: Any) -> float:
    """Return the principal angle of a rotation matrix in degrees."""
    matrix = np.asarray(rotation, dtype=float)
    cosine = np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


@dataclass(frozen=True)
class Sample:
    """One synchronized robot pose and target observation."""

    T_base_tool: np.ndarray
    T_camera_target: np.ndarray
    sample_id: str = ''
    timestamp: Optional[float] = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(self, 'T_base_tool', as_transform(self.T_base_tool, 'T_base_tool'))
        object.__setattr__(self, 'T_camera_target', as_transform(self.T_camera_target, 'T_camera_target'))
        if self.timestamp is not None and not math.isfinite(float(self.timestamp)):
            raise ValueError('timestamp must be finite')
        object.__setattr__(self, 'sample_id', str(self.sample_id))
        object.__setattr__(self, 'metadata', dict(self.metadata or {}))

    def to_dict(self):
        result = {
            'T_base_tool': self.T_base_tool.tolist(),
            'T_camera_target': self.T_camera_target.tolist(),
        }
        if self.sample_id:
            result['id'] = self.sample_id
        if self.timestamp is not None:
            result['timestamp'] = float(self.timestamp)
        if self.metadata:
            result['metadata'] = dict(self.metadata)
        return result

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, Mapping):
            raise ValueError('sample must be a mapping')
        return cls(
            value.get('T_base_tool', value.get('base_tool')),
            value.get('T_camera_target', value.get('camera_target')),
            value.get('id', value.get('sample_id', '')),
            value.get('timestamp'),
            value.get('metadata', {}),
        )


@dataclass(frozen=True)
class Metrics:
    sample_count: int
    mean_translation_m: tuple
    translation_std_m: tuple
    translation_rms_m: float
    rotation_mean_deg: float
    rotation_std_deg: float
    rotation_rms_deg: float
    translation_errors_m: tuple = ()
    rotation_errors_deg: tuple = ()
    outlier_indices: tuple = ()

    def to_dict(self):
        return {
            'sample_count': int(self.sample_count),
            'mean_translation_m': [
                float(value) for value in self.mean_translation_m
            ],
            'translation_std_m': [
                float(value) for value in self.translation_std_m
            ],
            'translation_rms_m': float(self.translation_rms_m),
            'rotation_mean_deg': float(self.rotation_mean_deg),
            'rotation_std_deg': float(self.rotation_std_deg),
            'rotation_rms_deg': float(self.rotation_rms_deg),
            'translation_errors_m': [
                float(value) for value in self.translation_errors_m
            ],
            'rotation_errors_deg': [
                float(value) for value in self.rotation_errors_deg
            ],
            'outlier_indices': [
                int(value) for value in self.outlier_indices
            ],
        }


@dataclass(frozen=True)
class Calibration:
    T_tool_camera: np.ndarray
    tool_frame: str
    camera_frame: str
    method: str
    metrics: Optional[Metrics]

    def __post_init__(self):
        object.__setattr__(self, 'T_tool_camera', as_transform(self.T_tool_camera, 'T_tool_camera'))
        if not self.tool_frame or not self.camera_frame:
            raise ValueError('calibration frame names must be non-empty')


def _samples(values: Iterable[Any]) -> list[Sample]:
    if isinstance(values, Sample):
        values = [values]
    result = []
    for value in values:
        result.append(value if isinstance(value, Sample) else Sample.from_dict(value))
    return result


def solve(samples: Iterable[Any], method: str = 'TSAI', backend: str = 'auto') -> np.ndarray:
    """Solve for ``T_tool_camera`` using OpenCV or the NumPy fallback."""
    values = _samples(samples)
    if len(values) < 3:
        raise ValueError('at least three samples are required')
    key = str(method).upper().replace('CALIB_HAND_EYE_', '')
    if key not in METHODS:
        raise ValueError(f'method must be one of {METHODS}')
    backend = str(backend).lower()
    if backend not in ('auto', 'opencv', 'numpy'):
        raise ValueError('backend must be auto, opencv, or numpy')
    if backend in ('auto', 'opencv'):
        try:
            import cv2
            constants = {
                'TSAI': cv2.CALIB_HAND_EYE_TSAI,
                'PARK': cv2.CALIB_HAND_EYE_PARK,
                'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
                'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
                'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
            }
            rotation, translation = cv2.calibrateHandEye(
                [s.T_base_tool[:3, :3] for s in values],
                [s.T_base_tool[:3, 3].reshape(3, 1) for s in values],
                [s.T_camera_target[:3, :3] for s in values],
                [s.T_camera_target[:3, 3].reshape(3, 1) for s in values],
                method=constants[key],
            )
            return _make_transform(rotation, translation)
        except (ImportError, AttributeError, RuntimeError):
            if backend == 'opencv':
                raise
    # Pairwise A X = X B.  This fallback is deterministic and dependency-light.
    rows = []
    pairs = []
    for i in range(len(values) - 1):
        for j in range(i + 1, len(values)):
            a = inverse(values[j].T_base_tool) @ values[i].T_base_tool
            b = values[j].T_camera_target @ inverse(values[i].T_camera_target)
            rows.append(np.kron(np.eye(3), a[:3, :3]) - np.kron(b[:3, :3].T, np.eye(3)))
            pairs.append((a, b))
    design = np.vstack(rows)
    _, singular, vh = np.linalg.svd(design)
    if singular[-1] > 1e-5 * max(singular[0], 1.0):
        raise ValueError('samples do not provide sufficiently diverse rotation')
    # The homogeneous nullspace vector is defined only up to a global sign.
    # Project both signs onto SO(3), then select the one that best satisfies
    # the original pairwise rotation equations.  A raw vector near -I can
    # otherwise be projected to an incorrect 180-degree rotation.
    raw_rotation = vh[-1].reshape((3, 3), order='F')
    candidates = (
        _project_rotation(raw_rotation),
        _project_rotation(-raw_rotation),
    )
    def rotation_residual(candidate):
        return sum(
            float(np.linalg.norm(
                a[:3, :3] @ candidate - candidate @ b[:3, :3]
            ))
            for a, b in pairs
        )
    rotation = min(candidates, key=rotation_residual)
    if rotation_residual(rotation) > 1e-4 * len(pairs):
        raise ValueError('samples do not provide a consistent rotation solution')
    left = np.vstack([a[:3, :3] - np.eye(3) for a, _ in pairs])
    right = np.concatenate([rotation @ b[:3, 3] - a[:3, 3] for a, b in pairs])
    translation, _, rank, _ = np.linalg.lstsq(left, right, rcond=None)
    if rank < 3:
        raise ValueError('samples do not constrain translation')
    return _make_transform(rotation, translation)


def validate(samples: Iterable[Any], T_tool_camera: Any,
             translation_outlier_m: Optional[float] = None,
             rotation_outlier_deg: Optional[float] = None) -> Metrics:
    values = _samples(samples)
    if not values:
        raise ValueError('at least one sample is required')
    transform = as_transform(T_tool_camera, 'T_tool_camera')
    implied = np.stack([s.T_base_tool @ transform @ s.T_camera_target for s in values])
    translations = implied[:, :3, 3]
    mean = np.mean(translations, axis=0)
    errors = np.linalg.norm(translations - mean, axis=1)
    mean_rotation = _project_rotation(np.mean(implied[:, :3, :3], axis=0))
    rotation_errors = np.asarray([
        rotation_angle_deg(mean_rotation.T @ item[:3, :3]) for item in implied
    ])
    outliers = set()
    if translation_outlier_m is not None:
        outliers.update(np.flatnonzero(errors > float(translation_outlier_m)).tolist())
    if rotation_outlier_deg is not None:
        outliers.update(np.flatnonzero(rotation_errors > float(rotation_outlier_deg)).tolist())
    return Metrics(
        len(values), tuple(mean), tuple(np.std(translations, axis=0)),
        float(np.sqrt(np.mean(errors ** 2))), float(np.mean(rotation_errors)),
        float(np.std(rotation_errors)), float(np.sqrt(np.mean(rotation_errors ** 2))),
        tuple(float(v) for v in errors), tuple(float(v) for v in rotation_errors),
        tuple(sorted(int(v) for v in outliers)),
    )


def _read(path):
    path = Path(path)
    text = path.read_text(encoding='utf-8')
    if path.suffix.lower() == '.json':
        return json.loads(text)
    if yaml is None:
        raise RuntimeError('PyYAML is required for YAML files')
    return yaml.safe_load(text)


def _write(path, document, overwrite=False):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not overwrite:
        raise FileExistsError(f'{path} exists; pass --overwrite explicitly')
    if path.suffix.lower() == '.json':
        payload = json.dumps(document, indent=2, ensure_ascii=True) + '\n'
    else:
        if yaml is None:
            raise RuntimeError('PyYAML is required for YAML files')
        payload = yaml.safe_dump(document, sort_keys=False)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.' + path.name + '.', suffix='.tmp',
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if overwrite:
            os.replace(temporary, path)
        else:
            os.link(temporary, path)
            temporary.unlink()
        temporary = None
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return str(path)


def save_samples(path, samples, overwrite=False):
    return _write(path, {
        'schema': SCHEMA_SAMPLES, 'version': SCHEMA_VERSION,
        'transform_convention': TRANSFORM_CONVENTION,
        'samples': [s.to_dict() for s in _samples(samples)],
    }, overwrite)


def load_samples(path):
    document = _read(path)
    if not isinstance(document, Mapping):
        raise ValueError('sample file must contain a mapping')
    if document.get('schema') not in (None, SCHEMA_SAMPLES):
        raise ValueError('unexpected sample schema')
    return _samples(document.get('samples', []))


def save_calibration(
        path, calibration, tool_frame='tool0',
        camera_frame='wrist_d405_color_optical_frame',
                     method='unknown', metrics=None, overwrite=False):
    if isinstance(calibration, Calibration):
        transform, method, metrics = calibration.T_tool_camera, calibration.method, calibration.metrics
        tool_frame, camera_frame = calibration.tool_frame, calibration.camera_frame
    else:
        transform = as_transform(calibration, 'T_tool_camera')
    return _write(path, {
        'schema': SCHEMA_CALIBRATION, 'version': SCHEMA_VERSION,
        'transform_convention': TRANSFORM_CONVENTION,
        'tool_frame': str(tool_frame), 'camera_frame': str(camera_frame),
        'T_tool_camera': as_transform(transform, 'T_tool_camera').tolist(),
        'method': str(method), 'validation': metrics.to_dict() if metrics else None,
    }, overwrite)


def load_calibration(path):
    document = _read(path)
    if not isinstance(document, Mapping) or document.get('schema') not in (None, SCHEMA_CALIBRATION):
        raise ValueError('unexpected calibration schema')
    raw_metrics = document.get('validation')
    metrics = None
    if isinstance(raw_metrics, Mapping):
        metrics = Metrics(
            int(raw_metrics.get('sample_count', 0)),
            tuple(raw_metrics.get('mean_translation_m', [0, 0, 0])),
            tuple(raw_metrics.get('translation_std_m', [0, 0, 0])),
            float(raw_metrics.get('translation_rms_m', 0)),
            float(raw_metrics.get('rotation_mean_deg', 0)),
            float(raw_metrics.get('rotation_std_deg', 0)),
            float(raw_metrics.get('rotation_rms_deg', 0)),
            tuple(raw_metrics.get('translation_errors_m', [])),
            tuple(raw_metrics.get('rotation_errors_deg', [])),
            tuple(raw_metrics.get('outlier_indices', [])),
        )
    return Calibration(
        document.get('T_tool_camera'), str(document.get('tool_frame', 'tool0')),
        str(document.get('camera_frame', 'wrist_d405_link')),
        str(document.get('method', 'unknown')), metrics,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description='Solve and save eye-in-hand calibration')
    parser.add_argument('--samples', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--tool-frame', default='tool0')
    parser.add_argument(
        '--camera-frame', default='wrist_d405_color_optical_frame'
    )
    parser.add_argument('--method', default='TSAI')
    parser.add_argument('--backend', default='auto', choices=['auto', 'opencv', 'numpy'])
    parser.add_argument('--minimum-samples', type=int, default=15)
    parser.add_argument('--max-translation-rms-m', type=float, default=0.01)
    parser.add_argument('--max-rotation-rms-deg', type=float, default=2.0)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args(argv)
    samples = load_samples(args.samples)
    if len(samples) < args.minimum_samples:
        raise SystemExit(f'need at least {args.minimum_samples} samples; got {len(samples)}')
    transform = solve(samples, args.method, args.backend)
    metrics = validate(samples, transform, args.max_translation_rms_m, args.max_rotation_rms_deg)
    print(json.dumps(metrics.to_dict(), indent=2))
    if metrics.translation_rms_m > args.max_translation_rms_m or metrics.rotation_rms_deg > args.max_rotation_rms_deg:
        raise SystemExit('validation thresholds failed; calibration was NOT saved')
    save_calibration(args.output, transform, args.tool_frame, args.camera_frame, args.method, metrics, args.overwrite)
    print(f'HAND_EYE_CALIBRATION_SAVED {Path(args.output).resolve()}')


def validate_main(argv=None):
    parser = argparse.ArgumentParser(description='Validate a saved eye-in-hand calibration')
    parser.add_argument('--samples', required=True)
    parser.add_argument('--calibration', required=True)
    args = parser.parse_args(argv)
    metrics = validate(load_samples(args.samples), load_calibration(args.calibration).T_tool_camera)
    print(json.dumps(metrics.to_dict(), indent=2))


__all__ = ['Calibration', 'Metrics', 'Sample', 'as_transform', 'inverse',
           'load_calibration', 'load_samples', 'main', 'rotation_angle_deg',
           'save_calibration', 'save_samples', 'solve', 'validate']

        
