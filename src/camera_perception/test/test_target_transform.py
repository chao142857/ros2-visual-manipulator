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

import math

from geometry_msgs.msg import Point, PointStamped, TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer

from camera_perception.target_transform_node import (
    point_is_finite,
    transform_point_stamped,
)


def test_point_is_finite_rejects_nan_and_infinity():
    assert point_is_finite(Point(x=1.0, y=2.0, z=3.0))
    assert not point_is_finite(Point(x=math.nan, y=0.0, z=0.0))
    assert not point_is_finite(Point(x=0.0, y=math.inf, z=0.0))


def test_transform_point_uses_tf2_and_preserves_observation_stamp():
    buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_frame'
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    transform.transform.rotation.z = math.sqrt(0.5)
    transform.transform.rotation.w = math.sqrt(0.5)
    buffer.set_transform_static(transform, 'test_authority')

    message = PointStamped()
    message.header.stamp = Time(seconds=5.0).to_msg()
    message.header.frame_id = 'camera_frame'
    message.point.x = 1.0

    result = transform_point_stamped(
        buffer,
        message,
        'base_link',
        timeout_sec=0.0,
    )

    assert result.header.frame_id == 'base_link'
    assert result.header.stamp == message.header.stamp
    assert math.isclose(result.point.x, 1.0, abs_tol=1.0e-9)
    assert math.isclose(result.point.y, 3.0, abs_tol=1.0e-9)
    assert math.isclose(result.point.z, 3.0, abs_tol=1.0e-9)
