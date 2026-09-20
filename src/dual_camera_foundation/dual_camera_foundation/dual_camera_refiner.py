"""Select a guarded wrist-camera refinement without point-cloud fusion.

The output is never an unconditional D405 override. A D405 point must agree
with a fresh Astra reference in frame, observation time, and position. The
selected output must also satisfy a bounded jump from the recent output.
These geometric gates are only a same-target proxy; upstream nodes must still
ensure that both topics describe the same physical target.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
from typing import Optional

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node


@dataclass
class _Observation:
    message: PointStamped
    received_at_sec: float
    stamp_sec: float


@dataclass(frozen=True)
class RefinementDecision:
    """Pure, testable result of the cross-camera consistency gates."""

    message: Optional[PointStamped]
    source: str
    reason: str


def finite(point):
    return all(math.isfinite(value) for value in (point.x, point.y, point.z))


def _stamp_seconds(stamp) -> Optional[float]:
    """Return a positive finite ROS stamp, or ``None`` for invalid time."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return None
    value = sec + nanosec / 1.0e9
    if value <= 0.0 or not math.isfinite(value):
        return None
    return value


def point_distance(first, second):
    """Return Euclidean distance between two finite PointStamped values."""
    if not finite(first.point) or not finite(second.point):
        return math.inf
    return math.sqrt(
        (first.point.x - second.point.x) ** 2
        + (first.point.y - second.point.y) ** 2
        + (first.point.z - second.point.z) ** 2
    )


def _same_frame(first, second):
    return (
        str(first.header.frame_id or '').strip()
        == str(second.header.frame_id or '').strip()
    )


def select_refined_source(
    astra,
    d405,
    last_output=None,
    *,
    allow_astra_fallback=True,
    require_astra_reference=True,
    require_same_frame=True,
    max_pair_time_delta_sec=0.25,
    max_pair_distance_m=0.08,
    max_output_jump_m=0.08,
):
    """Choose D405 only after cross-camera and recent-output checks.

    Inputs are assumed to have already passed per-source freshness checks.
    When D405 is rejected, a valid Astra message may be returned as the safe
    fallback. No averaging or semantic object association is attempted.
    """
    candidate = None
    source = ''
    rejection = 'D405 observation unavailable'

    if d405 is not None:
        if require_astra_reference and astra is None:
            rejection = 'D405 rejected: fresh Astra reference is required'
        elif astra is not None:
            if require_same_frame and not _same_frame(astra, d405):
                rejection = 'D405 rejected: camera outputs use different frames'
            else:
                astra_stamp = _stamp_seconds(astra.header.stamp)
                d405_stamp = _stamp_seconds(d405.header.stamp)
                if astra_stamp is None or d405_stamp is None:
                    rejection = 'D405 rejected: invalid observation timestamp'
                elif abs(d405_stamp - astra_stamp) > max_pair_time_delta_sec:
                    rejection = 'D405 rejected: cross-camera timestamp gap'
                elif point_distance(astra, d405) > max_pair_distance_m:
                    rejection = 'D405 rejected: cross-camera position mismatch'
                else:
                    candidate = d405
                    source = 'd405'
        else:
            candidate = d405
            source = 'd405'

    if candidate is None and allow_astra_fallback and astra is not None:
        candidate = astra
        source = 'astra'

    if candidate is None:
        return RefinementDecision(None, '', rejection)

    if last_output is not None:
        if require_same_frame and not _same_frame(last_output, candidate):
            return RefinementDecision(
                None,
                '',
                'candidate rejected: output frame changed',
            )
        if _same_frame(last_output, candidate):
            jump = point_distance(last_output, candidate)
            if jump > max_output_jump_m:
                # If a D405 switch caused the jump, retain a continuous Astra
                # fallback when possible instead of publishing the jump.
                if (
                    source == 'd405'
                    and allow_astra_fallback
                    and astra is not None
                    and _same_frame(last_output, astra)
                    and point_distance(last_output, astra)
                    <= max_output_jump_m
                ):
                    return RefinementDecision(
                        astra,
                        'astra',
                        'D405 rejected: output jump; using Astra fallback',
                    )
                return RefinementDecision(
                    None,
                    '',
                    'candidate rejected: output jump exceeds limit',
                )

    reason = 'D405 refinement accepted'
    if source == 'astra':
        reason = f'{rejection}; using Astra fallback'
    return RefinementDecision(candidate, source, reason)


class DualCameraRefiner(Node):
    """Publish a guarded D405 point or a fresh Astra fallback."""

    def __init__(self):
        super().__init__('dual_camera_refiner')
        defaults = {
            'astra_topic': '/perception/astra_target_point_base',
            'd405_topic': '/perception/d405_target_point_base',
            'output_topic': '/perception/dual_target_point_base',
            'astra_max_age_sec': 1.0,
            'd405_max_age_sec': 0.35,
            'future_tolerance_sec': 0.05,
            'max_pair_time_delta_sec': 0.25,
            'max_pair_distance_m': 0.08,
            'max_output_jump_m': 0.08,
            'output_jump_reference_max_age_sec': 1.0,
            'require_astra_reference': True,
            'require_same_frame': True,
            'allow_astra_fallback': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        value = self.get_parameter
        self.astra_max_age_sec = float(value('astra_max_age_sec').value)
        self.d405_max_age_sec = float(value('d405_max_age_sec').value)
        self.future_tolerance_sec = float(
            value('future_tolerance_sec').value
        )
        self.max_pair_time_delta_sec = float(
            value('max_pair_time_delta_sec').value
        )
        self.max_pair_distance_m = float(
            value('max_pair_distance_m').value
        )
        self.max_output_jump_m = float(
            value('max_output_jump_m').value
        )
        self.output_jump_reference_max_age_sec = float(
            value('output_jump_reference_max_age_sec').value
        )
        self.require_astra_reference = bool(
            value('require_astra_reference').value
        )
        self.require_same_frame = bool(value('require_same_frame').value)
        self.allow_astra_fallback = bool(
            value('allow_astra_fallback').value
        )
        positive_limits = (
            self.astra_max_age_sec,
            self.d405_max_age_sec,
            self.max_pair_time_delta_sec,
            self.max_pair_distance_m,
            self.max_output_jump_m,
            self.output_jump_reference_max_age_sec,
        )
        if any(not math.isfinite(item) or item <= 0.0 for item in positive_limits):
            raise ValueError('refinement limits must be finite and positive')
        if (
            not math.isfinite(self.future_tolerance_sec)
            or self.future_tolerance_sec < 0.0
        ):
            raise ValueError('future_tolerance_sec must be finite and non-negative')

        self.astra: Optional[_Observation] = None
        self.d405: Optional[_Observation] = None
        self.last_output: Optional[_Observation] = None
        self._last_decision_reason = ''
        self.publisher = self.create_publisher(
            PointStamped,
            value('output_topic').value,
            10,
        )
        self.create_subscription(
            PointStamped,
            value('astra_topic').value,
            self.astra_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            value('d405_topic').value,
            self.d405_callback,
            10,
        )
        self.timer = self.create_timer(0.05, self.publish_latest)

    def _accept(self, message: PointStamped) -> Optional[_Observation]:
        """Validate frame, finite point, timestamp, and future bound."""
        frame_id = str(message.header.frame_id or '').strip()
        stamp_sec = _stamp_seconds(message.header.stamp)
        if not frame_id or stamp_sec is None or not finite(message.point):
            return None
        received_at_sec = self.get_clock().now().nanoseconds / 1.0e9
        if not math.isfinite(received_at_sec) or received_at_sec <= 0.0:
            return None
        if stamp_sec - received_at_sec > self.future_tolerance_sec:
            return None
        return _Observation(
            copy.deepcopy(message),
            received_at_sec,
            stamp_sec,
        )

    def astra_callback(self, message):
        observation = self._accept(message)
        if observation is not None:
            self.astra = observation

    def d405_callback(self, message):
        observation = self._accept(message)
        if observation is not None:
            self.d405 = observation

    def _fresh(self, observation, max_age_sec, now_sec):
        if observation is None:
            return None
        receive_age = now_sec - observation.received_at_sec
        observation_age = now_sec - observation.stamp_sec
        if receive_age < 0.0 or observation_age < -self.future_tolerance_sec:
            return None
        if receive_age > max_age_sec or observation_age > max_age_sec:
            return None
        return observation

    def _report_decision(self, reason):
        if reason != self._last_decision_reason:
            self.get_logger().info(reason)
            self._last_decision_reason = reason

    def publish_latest(self):
        now_sec = self.get_clock().now().nanoseconds / 1.0e9
        d405 = self._fresh(self.d405, self.d405_max_age_sec, now_sec)
        astra = self._fresh(self.astra, self.astra_max_age_sec, now_sec)
        last_output = self._fresh(
            self.last_output,
            self.output_jump_reference_max_age_sec,
            now_sec,
        )
        decision = select_refined_source(
            astra.message if astra is not None else None,
            d405.message if d405 is not None else None,
            last_output.message if last_output is not None else None,
            allow_astra_fallback=self.allow_astra_fallback,
            require_astra_reference=self.require_astra_reference,
            require_same_frame=self.require_same_frame,
            max_pair_time_delta_sec=self.max_pair_time_delta_sec,
            max_pair_distance_m=self.max_pair_distance_m,
            max_output_jump_m=self.max_output_jump_m,
        )
        self._report_decision(decision.reason)
        if decision.message is None:
            return
        output = copy.deepcopy(decision.message)
        self.publisher.publish(output)
        stamp_sec = _stamp_seconds(output.header.stamp)
        if stamp_sec is not None:
            self.last_output = _Observation(
                copy.deepcopy(output),
                now_sec,
                stamp_sec,
            )


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraRefiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    'DualCameraRefiner',
    'RefinementDecision',
    '_stamp_seconds',
    'finite',
    'main',
    'point_distance',
    'select_refined_source',
]
