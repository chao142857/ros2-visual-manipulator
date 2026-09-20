# Copyright 2015 Open Source Robotics Foundation, Inc.
# Copyright 2026 chao (project modifications)
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

# Source: ros2/ros2cli tag 0.18.18, ros2pkg/ros2pkg/resource/ament_python/
# Template: test_pep257.py.em (Apache-2.0).
# https://github.com/ros2/ros2cli/tree/0.18.18/ros2pkg
# Existing project change: module docstring added.
# Attribution restored in 2026; existing test behavior is unchanged.

"""ROS package docstring style test."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
