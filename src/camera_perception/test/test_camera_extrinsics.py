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

import pytest
import yaml

from camera_perception.camera_extrinsics import (
    CameraExtrinsics,
    save_camera_extrinsics,
)


def test_camera_extrinsics_normalizes_quaternion():
    extrinsics = CameraExtrinsics.from_values(
        'base_link',
        'camera_link',
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0, 2.0],
    )

    assert extrinsics.quaternion == (0.0, 0.0, 0.0, 1.0)


@pytest.mark.parametrize(
    'parent, child, xyz, quaternion',
    [
        ('', 'camera_link', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        (
            'base_link',
            'base_link',
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ),
        (
            'base_link',
            'camera_link',
            [math.nan, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ),
        (
            'base_link',
            'camera_link',
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ),
    ],
)
def test_camera_extrinsics_rejects_invalid_values(
    parent,
    child,
    xyz,
    quaternion,
):
    with pytest.raises(ValueError):
        CameraExtrinsics.from_values(parent, child, xyz, quaternion)


def test_save_requires_explicit_overwrite(tmp_path):
    path = tmp_path / 'camera_extrinsics.yaml'
    first = CameraExtrinsics.from_values(
        'base_link',
        'camera_link',
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    )
    second = CameraExtrinsics.from_values(
        'base_link',
        'camera_link',
        [4.0, 5.0, 6.0],
        [0.0, 0.0, 0.0, 1.0],
    )

    save_camera_extrinsics(path, first)
    with pytest.raises(FileExistsError):
        save_camera_extrinsics(path, second)

    unchanged = yaml.safe_load(path.read_text(encoding='utf-8'))
    unchanged_parameters = unchanged[
        'camera_extrinsics_publisher_node'
    ]['ros__parameters']
    assert unchanged_parameters['xyz'] == [1.0, 2.0, 3.0]

    save_camera_extrinsics(path, second, overwrite=True)
    replaced = yaml.safe_load(path.read_text(encoding='utf-8'))
    replaced_parameters = replaced[
        'camera_extrinsics_publisher_node'
    ]['ros__parameters']
    assert replaced_parameters['xyz'] == [4.0, 5.0, 6.0]
