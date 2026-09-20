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

"""Launch a namespaced wrist D405 with independent health diagnostics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from wrist_camera_bringup.health import normalize_serial_no
from wrist_camera_bringup.launch_support import (
    build_realsense_arguments,
    parse_bool,
)


def _launch_setup(context):
    """Resolve, validate, and compose runtime launch actions."""
    namespace = LaunchConfiguration('camera_namespace').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    serial_input = LaunchConfiguration('serial_no').perform(context)
    serial = normalize_serial_no(serial_input)
    start_camera = parse_bool(
        LaunchConfiguration('start_camera').perform(context),
        'start_camera',
    )
    start_health = parse_bool(
        LaunchConfiguration('start_health_monitor').perform(context),
        'start_health_monitor',
    )
    publish_placeholder = parse_bool(
        LaunchConfiguration(
            'publish_uncalibrated_mount_tf'
        ).perform(context),
        'publish_uncalibrated_mount_tf',
    )
    require_mount_tf = parse_bool(
        LaunchConfiguration('require_mount_tf').perform(context),
        'require_mount_tf',
    )
    driver_arguments = build_realsense_arguments(
        namespace,
        camera_name,
        serial,
        LaunchConfiguration('depth_profile').perform(context),
        LaunchConfiguration('color_profile').perform(context),
        LaunchConfiguration('reconnect_timeout').perform(context),
        LaunchConfiguration('diagnostics_period').perform(context),
        LaunchConfiguration('align_depth').perform(context),
        LaunchConfiguration('pointcloud').perform(context),
    )
    namespace = driver_arguments['camera_namespace']
    camera_name = driver_arguments['camera_name']
    camera_base = f'/{namespace}/{camera_name}'
    mount_child_frame = f'{camera_name}_link'
    package_share = get_package_share_directory('wrist_camera_bringup')
    health_config = os.path.join(
        package_share, 'config', 'wrist_d405_health.yaml'
    )

    actions = []
    if start_camera:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py',
                    ])
                ),
                launch_arguments=driver_arguments.items(),
            )
        )
    if start_health:
        actions.append(
            Node(
                package='wrist_camera_bringup',
                executable='wrist_d405_health_monitor',
                namespace=namespace,
                name='wrist_d405_health_monitor',
                output='screen',
                parameters=[
                    health_config,
                    {
                        'camera_namespace': namespace,
                        'camera_name': camera_name,
                        'serial_no': serial,
                        'diagnostics_output_topic': (
                            f'{camera_base}/health/diagnostics'
                        ),
                        'mount_child_frame': mount_child_frame,
                        'require_mount_tf': require_mount_tf,
                        'mount_tf_is_placeholder': publish_placeholder,
                        'require_aligned_depth': (
                            driver_arguments['align_depth.enable'] == 'true'
                        ),
                    },
                ],
            )
        )
    if publish_placeholder:
        actions.extend([
            LogInfo(msg=(
                'WARNING: publishing identity tool0 -> '
                f'{mount_child_frame} as an UNCALIBRATED placeholder; '
                'it must not be used for hand-eye geometry.'
            )),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='wrist_d405_uncalibrated_mount_tf',
                output='screen',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'tool0',
                    '--child-frame-id', mount_child_frame,
                ],
            ),
        ])
    return actions


def generate_launch_description():
    """Build the wrist D405 launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='Start the upstream RealSense D405 driver.',
        ),
        DeclareLaunchArgument(
            'start_health_monitor', default_value='true',
            description='Start independent wrist-camera diagnostics.',
        ),
        DeclareLaunchArgument(
            'camera_namespace', default_value='wrist_camera',
            description='Namespace above the unique D405 camera name.',
        ),
        DeclareLaunchArgument(
            'camera_name', default_value='wrist_d405',
            description='Unique D405 node, topic, and TF prefix.',
        ),
        DeclareLaunchArgument(
            'serial_no', default_value='1',
            description=(
                'D405 serial as plain digits. Defaults to the currently '
                'identified wrist unit; empty is allowed only for bench use.'
            ),
        ),
        DeclareLaunchArgument(
            'depth_profile', default_value='640x480x30',
            description='Conservative D405 depth profile.',
        ),
        DeclareLaunchArgument(
            'color_profile', default_value='640x480x30',
            description='Conservative D405 depth-module color profile.',
        ),
        DeclareLaunchArgument(
            'reconnect_timeout', default_value='3.0',
            description='Seconds between upstream reconnect attempts.',
        ),
        DeclareLaunchArgument(
            'diagnostics_period', default_value='1.0',
            description='Upstream driver diagnostic period in seconds.',
        ),
        DeclareLaunchArgument(
            'align_depth', default_value='false',
            description='Enable upstream depth-to-color alignment.',
        ),
        DeclareLaunchArgument(
            'pointcloud', default_value='false',
            description='Enable upstream point-cloud generation.',
        ),
        DeclareLaunchArgument(
            'require_mount_tf', default_value='false',
            description=(
                'Treat a missing calibrated tool0-to-camera TF as ERROR.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_uncalibrated_mount_tf', default_value='false',
            description=(
                'Publish an identity placeholder only for visualization; '
                'never valid for hand-eye geometry.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
