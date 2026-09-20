"""Pure validation helpers for the experimental visual grasp controller."""

import math

import numpy as np


def _vector3(value):
    try:
        vector = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None
    return vector


def point_in_workspace(point, workspace_min, workspace_max):
    """Return whether a finite 3D point is inside the closed bounds."""
    point = _vector3(point)
    lower = _vector3(workspace_min)
    upper = _vector3(workspace_max)
    if point is None or lower is None or upper is None:
        return False
    if not np.all(lower < upper):
        return False
    return bool(np.all(point >= lower) and np.all(point <= upper))


def target_sample_span(samples):
    """Return the largest pairwise distance, or infinity for bad input."""
    try:
        points = np.asarray(samples, dtype=np.float64)
    except (TypeError, ValueError):
        return math.inf
    if (
        points.ndim != 2
        or points.shape[0] == 0
        or points.shape[1] != 3
        or not np.all(np.isfinite(points))
    ):
        return math.inf
    differences = points[:, None, :] - points[None, :, :]
    return float(np.max(np.linalg.norm(differences, axis=2)))


def stable_median_target(
    samples,
    required_count,
    maximum_span,
    workspace_min,
    workspace_max,
):
    """Lock the coordinate-wise median of a compact recent sample set."""
    try:
        points = np.asarray(samples, dtype=np.float64)
    except (TypeError, ValueError):
        return None, math.inf
    if (
        not isinstance(required_count, int)
        or required_count < 1
        or points.ndim != 2
        or points.shape[1:] != (3,)
        or points.shape[0] < required_count
    ):
        return None, math.inf

    recent = points[-required_count:]
    span = target_sample_span(recent)
    if (
        not math.isfinite(maximum_span)
        or maximum_span <= 0.0
        or span > maximum_span
    ):
        return None, span

    target = np.median(recent, axis=0)
    if not point_in_workspace(target, workspace_min, workspace_max):
        return None, span
    return target, span


def goal_reached(
    execute_hardware,
    current_error,
    ik_position_error,
    ik_approach_error_deg,
    arrival_tolerance,
    ik_position_tolerance,
    ik_approach_tolerance_deg,
):
    """Use measured TCP error for hardware and IK residuals for preview."""
    if execute_hardware:
        return bool(
            math.isfinite(current_error)
            and current_error <= arrival_tolerance
        )
    return bool(
        math.isfinite(ik_position_error)
        and math.isfinite(ik_approach_error_deg)
        and ik_position_error <= ik_position_tolerance
        and ik_approach_error_deg <= ik_approach_tolerance_deg
    )


def validate_gripper_configuration(
    open_pulse_us,
    close_pulse_us,
):
    """Require only in-range integer endpoint pulses."""
    try:
        values = (float(open_pulse_us), float(close_pulse_us))
    except (TypeError, ValueError) as error:
        raise ValueError('gripper pulses must be finite integers') from error
    if not all(
        math.isfinite(value) and value.is_integer() and value > 0.0
        for value in values
    ):
        raise ValueError('gripper pulses must be positive finite integers')
    open_value, close_value = (int(value) for value in values)
    if not all(600 <= value <= 1700 for value in (open_value, close_value)):
        raise ValueError('gripper pulses must be between 600 and 1700 us')
    return open_value, close_value
