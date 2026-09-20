import pytest
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from tabletop_grasp.joint_feedback import JointFeedback
from tabletop_grasp.moveit_adapter import ARM_JOINTS


def msg(names, values, seconds):
    return JointState(header=Header(stamp=Time(seconds=seconds).to_msg()),
                      name=list(names), position=list(values))


def test_passive_joint_is_required_and_partial_messages_are_merged_by_name():
    state = JointFeedback(1., 0.05)
    assert state.update(msg(reversed(ARM_JOINTS), [0.] * 6, 10), 10_000_000_000) is None
    result = state.update(msg(['tcp_extension_joint'], [0.026], 10), 10_000_000_000)
    assert result.name == list(ARM_JOINTS) + ['tcp_extension_joint']
    assert result.position[-1] == 0.026
    assert state.update(msg(ARM_JOINTS, [0.] * 6, 12), 12_000_000_000) is None


def test_invalid_feedback_clears_cached_joint_values():
    state = JointFeedback(1., 0.05)
    state.update(msg(ARM_JOINTS, [0.] * 6, 10), 10_000_000_000)
    with pytest.raises(ValueError):
        state.update(msg(['joint1', 'joint1'], [0., 0.], 10), 10_000_000_000)
    assert state.values == {}
