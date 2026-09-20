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

"""Unit tests for launch input validation and upstream argument mapping."""

from pathlib import Path
import re

import pytest

from wrist_camera_bringup.health import (
    normalize_serial_no,
    serial_for_realsense_launch,
)
from wrist_camera_bringup.launch_support import (
    build_realsense_arguments,
    normalize_namespace,
    normalize_node_name,
    parse_bool,
)


PACKAGE = Path(__file__).parents[1]


@pytest.mark.parametrize('value', ['', "''", '""', '   '])
def test_empty_serial_stays_unpinned(value):
    assert normalize_serial_no(value) == ''
    assert serial_for_realsense_launch(value) == "''"


@pytest.mark.parametrize(
    ('value', 'plain', 'upstream'),
    [
        ('1', '1', '_1'),
        (1, '1', '_1'),
        ('_0', '0', '_0'),
        (' 1 ', '1', '_1'),
    ],
)
def test_numeric_serial_is_normalized_for_upstream(
    value,
    plain,
    upstream,
):
    assert normalize_serial_no(value) == plain
    assert serial_for_realsense_launch(value) == upstream


@pytest.mark.parametrize(
    'value',
    ['abc', '230-322', '__1', '_', '+1'],
)
def test_invalid_serial_fails_fast(value):
    with pytest.raises(ValueError, match='serial_no'):
        normalize_serial_no(value)


def test_names_and_launch_booleans_are_validated():
    assert normalize_namespace('/wrist_camera/front/') == (
        'wrist_camera/front'
    )
    assert normalize_node_name('wrist_d405') == 'wrist_d405'
    assert parse_bool('YES', 'enabled') is True
    assert parse_bool('off', 'enabled') is False
    with pytest.raises(ValueError, match='namespace'):
        normalize_namespace('wrist-camera')
    with pytest.raises(ValueError, match='camera_name'):
        normalize_node_name('405camera')
    with pytest.raises(ValueError, match='enabled'):
        parse_bool('sometimes', 'enabled')


def test_upstream_arguments_are_conservative_and_console_compatible():
    arguments = build_realsense_arguments(
        '/wrist_camera/',
        'wrist_d405',
        '1',
    )
    assert arguments['camera_namespace'] == 'wrist_camera'
    assert arguments['camera_name'] == 'wrist_d405'
    assert arguments['serial_no'] == '_1'
    assert arguments['device_type'] == 'd405'
    assert arguments['depth_module.depth_profile'] == '640x480x30'
    assert arguments['depth_module.color_profile'] == '640x480x30'
    assert arguments['depth_module.depth_format'] == 'Z16'
    assert arguments['depth_module.color_format'] == 'RGB8'
    assert arguments['initial_reset'] == 'false'
    assert arguments['enable_sync'] == 'true'
    assert arguments['reconnect_timeout'] == '3.0'
    assert arguments['diagnostics_period'] == '1.0'
    for disabled in (
        'enable_infra',
        'enable_infra1',
        'enable_infra2',
        'enable_gyro',
        'enable_accel',
        'enable_rgbd',
        'pointcloud.enable',
        'align_depth.enable',
    ):
        assert arguments[disabled] == 'false'


def test_optional_alignment_and_pointcloud_are_forwarded():
    arguments = build_realsense_arguments(
        'wrist_camera',
        'wrist_d405',
        '',
        align_depth='true',
        pointcloud='yes',
    )
    assert arguments['align_depth.enable'] == 'true'
    assert arguments['pointcloud.enable'] == 'true'


def test_invalid_optional_filter_boolean_fails_fast():
    with pytest.raises(ValueError, match='align_depth'):
        build_realsense_arguments(
            'wrist_camera',
            'wrist_d405',
            '',
            align_depth='maybe',
        )


def test_unpinned_upstream_argument_uses_official_empty_sentinel():
    arguments = build_realsense_arguments(
        'wrist_camera',
        'wrist_d405',
        '',
    )
    assert arguments['serial_no'] == "''"


def test_console_wrapper_forwards_command_authority_defaults():
    text = (
        PACKAGE / 'launch' / 'wrist_d405_console.launch.py'
    ).read_text(encoding='utf-8')
    assert re.search(
        r"'arm_allow_motion_commands',\s*default_value='false'",
        text,
    )
    assert re.search(
        r"'arm_allow_motion_commands':\s*LaunchConfiguration\(\s*"
        r"'arm_allow_motion_commands'\s*\)",
        text,
    )
