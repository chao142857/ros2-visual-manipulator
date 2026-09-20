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

"""Launch the wrist D405 and Arm Console in one command."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose wrist camera, optional eye Astra, and existing Console."""
    wrist_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wrist_camera_bringup'),
                'launch',
                'wrist_d405.launch.py',
            ])
        ),
        launch_arguments={
            'start_camera': LaunchConfiguration('start_wrist_camera'),
            'start_health_monitor': LaunchConfiguration(
                'start_wrist_health_monitor'
            ),
            'camera_namespace': LaunchConfiguration(
                'wrist_camera_namespace'
            ),
            'camera_name': LaunchConfiguration('wrist_camera_name'),
            'serial_no': LaunchConfiguration('wrist_serial_no'),
            'depth_profile': LaunchConfiguration('wrist_depth_profile'),
            'color_profile': LaunchConfiguration('wrist_color_profile'),
            'reconnect_timeout': LaunchConfiguration(
                'wrist_reconnect_timeout'
            ),
            'diagnostics_period': LaunchConfiguration(
                'wrist_diagnostics_period'
            ),
            'align_depth': LaunchConfiguration('wrist_align_depth'),
            'pointcloud': LaunchConfiguration('wrist_pointcloud'),
            'require_mount_tf': LaunchConfiguration('require_mount_tf'),
            'publish_uncalibrated_mount_tf': LaunchConfiguration(
                'publish_uncalibrated_mount_tf'
            ),
        }.items(),
    )

    eye_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch',
                'astra_pro.launch.xml',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_eye_camera')),
        launch_arguments={
            'camera_name': 'camera',
            'serial_number': LaunchConfiguration('eye_camera_serial_number'),
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'tf_publish_rate': '0.0',
            'oni_log_level': 'warning',
            'oni_log_to_console': 'false',
            'oni_log_to_file': 'false',
        }.items(),
    )

    console = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_console'),
                'launch',
                'arm_console.launch.py',
            ])
        ),
        launch_arguments={
            'start_driver': LaunchConfiguration('start_driver'),
            'use_mock': LaunchConfiguration('use_mock'),
            'arm_allow_motion_commands': LaunchConfiguration(
                'arm_allow_motion_commands'
            ),
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'poll_period': LaunchConfiguration('poll_period'),
            'gripper_allow_hardware_actuation': LaunchConfiguration(
                'gripper_allow_hardware_actuation'
            ),
            'start_gripper_driver': LaunchConfiguration(
                'start_gripper_driver'
            ),
            'gripper_serial_port': LaunchConfiguration(
                'gripper_serial_port'
            ),
            'gripper_baud_rate': LaunchConfiguration(
                'gripper_baud_rate'
            ),
            'gripper_channel': LaunchConfiguration('gripper_channel'),
            'gripper_open_pulse_us': LaunchConfiguration(
                'gripper_open_pulse_us'
            ),
            'gripper_close_pulse_us': LaunchConfiguration(
                'gripper_close_pulse_us'
            ),
            'gripper_move_duration_ms': LaunchConfiguration(
                'gripper_move_duration_ms'
            ),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_wrist_camera', default_value='true',
            description='Start the wrist D405 driver.',
        ),
        DeclareLaunchArgument(
            'start_wrist_health_monitor', default_value='true',
            description='Start wrist D405 health diagnostics.',
        ),
        DeclareLaunchArgument(
            'wrist_camera_namespace', default_value='wrist_camera',
        ),
        DeclareLaunchArgument(
            'wrist_camera_name', default_value='wrist_d405',
        ),
        DeclareLaunchArgument(
            'wrist_serial_no', default_value='1',
            description=(
                'Plain numeric D405 serial; defaults to the identified '
                'wrist unit and remains overrideable.'
            ),
        ),
        DeclareLaunchArgument(
            'wrist_depth_profile', default_value='640x480x30',
        ),
        DeclareLaunchArgument(
            'wrist_color_profile', default_value='640x480x30',
        ),
        DeclareLaunchArgument(
            'wrist_reconnect_timeout', default_value='3.0',
        ),
        DeclareLaunchArgument(
            'wrist_diagnostics_period', default_value='1.0',
        ),
        DeclareLaunchArgument(
            'wrist_align_depth', default_value='false',
        ),
        DeclareLaunchArgument(
            'wrist_pointcloud', default_value='false',
        ),
        DeclareLaunchArgument(
            'require_mount_tf', default_value='false',
        ),
        DeclareLaunchArgument(
            'publish_uncalibrated_mount_tf', default_value='false',
        ),
        DeclareLaunchArgument(
            'start_eye_camera', default_value='false',
            description=(
                'Optionally start the Astra eye camera; false leaves it '
                'external and unmanaged.'
            ),
        ),
        DeclareLaunchArgument(
            'eye_camera_serial_number', default_value='',
        ),
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument(
            'arm_allow_motion_commands',
            default_value='false',
            description=(
                'Arm motion authority passed to Arm Console.'
            ),
        ),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0'
        ),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('poll_period', default_value='0.1'),
        DeclareLaunchArgument(
            'gripper_allow_hardware_actuation', default_value='false',
            description='Explicit opt-in to open gripper serial and accept PWM commands.',
        ),
        DeclareLaunchArgument(
            'start_gripper_driver', default_value='false'
        ),
        DeclareLaunchArgument(
            'gripper_serial_port',
            default_value=(
                '/dev/serial/by-id/CONFIGURE_GRIPPER_PORT'
            ),
        ),
        DeclareLaunchArgument(
            'gripper_baud_rate', default_value='115200'
        ),
        DeclareLaunchArgument('gripper_channel', default_value='0'),
        DeclareLaunchArgument(
            'gripper_open_pulse_us',
            default_value='600',
            description='Measured fully-open gripper pulse.',
        ),
        DeclareLaunchArgument(
            'gripper_close_pulse_us',
            default_value='1300',
            description='Measured object-gripping pulse.',
        ),
        DeclareLaunchArgument(
            'gripper_move_duration_ms', default_value='600'
        ),
        wrist_camera,
        eye_camera,
        console,
    ])
