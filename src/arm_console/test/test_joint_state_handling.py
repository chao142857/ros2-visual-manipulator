import math

import pytest

from arm_console.ros_bridge import FeedbackTracker, extract_joint_positions_degrees


def test_joint_state_is_reordered_by_name():
    names = ['joint3', 'joint1', 'joint6', 'joint2', 'joint5', 'joint4']
    positions = [0.3, 0.1, 0.6, 0.2, 0.5, 0.4]

    result = extract_joint_positions_degrees(names, positions)

    expected = tuple(math.degrees(value) for value in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    assert result == pytest.approx(expected)


def test_joint_state_can_include_future_extra_joints():
    names = [
        'joint1',
        'joint2',
        'gripper_joint',
        'joint3',
        'joint4',
        'joint5',
        'joint6',
    ]
    positions = [0.0, 0.1, 0.7, 0.2, 0.3, 0.4, 0.5]

    result = extract_joint_positions_degrees(names, positions)

    assert len(result) == 6
    assert result[-1] == pytest.approx(math.degrees(0.5))


@pytest.mark.parametrize(
    ('names', 'positions'),
    [
        (['joint1'], []),
        (['joint1'] * 6, [0.0] * 6),
        (
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'other'],
            [0.0] * 6,
        ),
        (
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            [0.0, 0.0, 0.0, 0.0, math.nan, 0.0],
        ),
    ],
)
def test_invalid_joint_states_are_rejected(names, positions):
    with pytest.raises(ValueError):
        extract_joint_positions_degrees(names, positions)


def test_feedback_requires_three_continuous_valid_frames_to_be_ready():
    tracker = FeedbackTracker(timeout_sec=1.0, recovery_frames_required=3)

    assert tracker.snapshot(10.0) == (None, 'never', 0)
    tracker.record_valid(10.0)
    assert tracker.snapshot(10.0) == (0.0, 'recovering', 1)
    tracker.record_valid(10.1)
    assert tracker.snapshot(10.1) == (0.0, 'recovering', 2)
    tracker.record_valid(10.2)
    assert tracker.snapshot(10.2) == (0.0, 'fresh', 3)


def test_feedback_loss_relocks_and_restarts_three_frame_recovery():
    tracker = FeedbackTracker(timeout_sec=1.0, recovery_frames_required=3)
    for received_at in (10.0, 10.1, 10.2):
        tracker.record_valid(received_at)

    age, stream, streak = tracker.snapshot(11.21)
    assert age == pytest.approx(1.01)
    assert (stream, streak) == ('stale', 0)

    tracker.record_valid(11.22)
    assert tracker.snapshot(11.22) == (0.0, 'recovering', 1)
    tracker.record_valid(11.32)
    assert tracker.snapshot(11.32) == (0.0, 'recovering', 2)
    tracker.record_valid(11.42)
    assert tracker.snapshot(11.42) == (0.0, 'fresh', 3)


def test_invalid_frame_breaks_valid_recovery_streak():
    tracker = FeedbackTracker(timeout_sec=1.0, recovery_frames_required=3)
    tracker.record_valid(10.0)
    tracker.record_valid(10.1)
    tracker.record_invalid()
    tracker.record_valid(10.2)

    assert tracker.snapshot(10.2) == (0.0, 'recovering', 1)


@pytest.mark.parametrize('timeout_sec', [math.nan, math.inf])
def test_feedback_tracker_rejects_non_finite_timeout(timeout_sec):
    with pytest.raises(
        ValueError,
        match='feedback timeout must be finite and positive',
    ):
        FeedbackTracker(timeout_sec=timeout_sec)
