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

"""Pure launch-input validation for the wrist D405."""

import re

from wrist_camera_bringup.health import serial_for_realsense_launch


ROS_NAME_PART = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def normalize_namespace(value):
    """Normalize a relative ROS namespace and reject invalid segments."""
    namespace = str(value).strip().strip('/')
    parts = namespace.split('/') if namespace else []
    if not parts or any(ROS_NAME_PART.fullmatch(part) is None for part in parts):
        raise ValueError('camera_namespace is not a valid ROS namespace')
    return '/'.join(parts)


def normalize_node_name(value):
    """Validate one ROS node/name segment."""
    name = str(value).strip()
    if ROS_NAME_PART.fullmatch(name) is None:
        raise ValueError('camera_name is not a valid ROS name')
    return name


def parse_bool(value, name):
    """Parse a launch-style boolean with explicit accepted spellings."""
    normalized = str(value).strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false')


def build_realsense_arguments(
    camera_namespace,
    camera_name,
    serial_no,
    depth_profile='640x480x30',
    color_profile='640x480x30',
    reconnect_timeout='3.0',
    diagnostics_period='1.0',
    align_depth='false',
    pointcloud='false',
):
    """Build conservative arguments for the upstream rs_launch.py."""
    namespace = normalize_namespace(camera_namespace)
    name = normalize_node_name(camera_name)
    align_depth_enabled = parse_bool(align_depth, 'align_depth')
    pointcloud_enabled = parse_bool(pointcloud, 'pointcloud')
    return {
        'camera_namespace': namespace,
        'camera_name': name,
        'serial_no': serial_for_realsense_launch(serial_no),
        'device_type': 'd405',
        'initial_reset': 'false',
        'enable_color': 'true',
        'enable_depth': 'true',
        'enable_infra': 'false',
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'enable_gyro': 'false',
        'enable_accel': 'false',
        'depth_module.depth_profile': str(depth_profile),
        'depth_module.depth_format': 'Z16',
        'depth_module.color_profile': str(color_profile),
        'depth_module.color_format': 'RGB8',
        'enable_sync': 'true',
        'enable_rgbd': 'false',
        'pointcloud.enable': str(pointcloud_enabled).lower(),
        'align_depth.enable': str(align_depth_enabled).lower(),
        'colorizer.enable': 'false',
        'decimation_filter.enable': 'false',
        'spatial_filter.enable': 'false',
        'temporal_filter.enable': 'false',
        'hole_filling_filter.enable': 'false',
        'publish_tf': 'true',
        'tf_publish_rate': '0.0',
        'wait_for_device_timeout': '-1.0',
        'reconnect_timeout': str(reconnect_timeout),
        'diagnostics_period': str(diagnostics_period),
        'log_level': 'info',
        'output': 'screen',
    }
