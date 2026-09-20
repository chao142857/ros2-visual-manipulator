import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped

from dual_camera_foundation.target_pose_transform import transform_pose_stamped
from dual_camera_foundation.visual_ik_node import quaternion_to_rotation
from dual_camera_foundation.visual_ik_node import rpy_to_rotation


def _quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class _Buffer:
    def __init__(self, transform):
        self.transform = transform
        self.requests = []

    def lookup_transform(self, target, source, stamp, timeout):
        self.requests.append((target, source, stamp, timeout))
        return self.transform


def test_transform_pose_stamped_applies_base_translation_and_orientation():
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    tx, ty, tz, tw = _quaternion_from_yaw(math.pi / 2.0)
    transform.transform.rotation.x = tx
    transform.transform.rotation.y = ty
    transform.transform.rotation.z = tz
    transform.transform.rotation.w = tw
    buffer = _Buffer(transform)

    message = PoseStamped()
    message.header.frame_id = 'camera_frame'
    message.header.stamp = Time(sec=7, nanosec=8)
    message.pose.position.x = 2.0
    sx, sy, sz, sw = _quaternion_from_yaw(math.radians(30.0))
    message.pose.orientation.x = sx
    message.pose.orientation.y = sy
    message.pose.orientation.z = sz
    message.pose.orientation.w = sw

    result = transform_pose_stamped(buffer, message, 'base_link')

    assert result.header.frame_id == 'base_link'
    assert result.header.stamp == message.header.stamp
    assert result.pose.position.x == pytest.approx(1.0)
    assert result.pose.position.y == pytest.approx(4.0)
    assert result.pose.position.z == pytest.approx(3.0)
    result_rotation = quaternion_to_rotation(
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    )
    expected_rotation = rpy_to_rotation(0.0, 0.0, math.radians(120.0))
    assert result_rotation == pytest.approx(expected_rotation)
    assert buffer.requests[0][0:2] == ('base_link', 'camera_frame')
