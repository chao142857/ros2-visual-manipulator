from geometry_msgs.msg import PointStamped

from dual_camera_foundation.dual_camera_refiner import (
    point_distance,
    select_refined_source,
)


def _point(x, y=0.0, z=0.3, stamp=10.0, frame='base_link'):
    message = PointStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int(
        round((stamp - int(stamp)) * 1_000_000_000)
    )
    message.point.x = x
    message.point.y = y
    message.point.z = z
    return message


def test_d405_is_selected_only_when_it_agrees_with_fresh_astra_reference():
    astra = _point(0.40, stamp=10.00)
    d405 = _point(0.43, stamp=10.10)
    decision = select_refined_source(astra, d405)
    assert decision.source == 'd405'
    assert decision.message is d405
    assert point_distance(astra, d405) < 0.08


def test_d405_without_astra_reference_fails_closed():
    decision = select_refined_source(None, _point(0.4))
    assert decision.message is None
    assert 'Astra reference' in decision.reason


def test_position_mismatch_uses_astra_fallback_instead_of_d405():
    astra = _point(0.30)
    d405 = _point(0.60)
    decision = select_refined_source(
        astra,
        d405,
        max_pair_distance_m=0.08,
    )
    assert decision.source == 'astra'
    assert decision.message is astra
    assert 'position mismatch' in decision.reason


def test_frame_or_timestamp_mismatch_never_overrides_astra():
    astra = _point(0.30, stamp=10.0)
    wrong_frame = _point(0.31, stamp=10.0, frame='camera_link')
    decision = select_refined_source(astra, wrong_frame)
    assert decision.source == 'astra'
    assert 'different frames' in decision.reason

    late = _point(0.31, stamp=11.0)
    decision = select_refined_source(
        astra,
        late,
        max_pair_time_delta_sec=0.25,
    )
    assert decision.source == 'astra'
    assert 'timestamp gap' in decision.reason


def test_output_jump_keeps_continuous_astra_fallback():
    last_output = _point(0.30, stamp=9.9)
    astra = _point(0.34, stamp=10.0)
    d405 = _point(0.40, stamp=10.0)
    decision = select_refined_source(
        astra,
        d405,
        last_output,
        max_pair_distance_m=0.08,
        max_output_jump_m=0.05,
    )
    assert decision.source == 'astra'
    assert decision.message is astra
    assert 'output jump' in decision.reason


def test_output_jump_without_safe_fallback_publishes_nothing():
    last_output = _point(0.10, stamp=9.9)
    astra = _point(0.40, stamp=10.0)
    decision = select_refined_source(
        astra,
        None,
        last_output,
        max_output_jump_m=0.08,
    )
    assert decision.message is None
    assert 'output jump' in decision.reason
