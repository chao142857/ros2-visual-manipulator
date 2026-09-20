"""Launch measured-state and isolated target-preview robot visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build RViz without publishing target values to the real state topic."""
    description_share = get_package_share_directory('arm_description')
    console_share = get_package_share_directory('arm_console')

    urdf_path = os.path.join(
        description_share,
        'urdf',
        'dummy-v2.urdf',
    )
    rviz_config_path = os.path.join(
        console_share,
        'config',
        'arm_console.rviz',
    )
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    start_physical = LaunchConfiguration(
        'start_physical_state_publisher'
    )
    start_rviz = LaunchConfiguration('start_rviz')

    physical_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(start_physical),
        parameters=[{'robot_description': robot_description}],
    )

    preview_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_preview_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': 'preview/',
            'ignore_timestamp': True,
        }],
        remappings=[
            ('joint_states', '/arm/preview_joint_states'),
        ],
    )

    preview_anchor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arm_preview_anchor',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'preview/base_link',
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_physical_state_publisher',
            default_value='true',
            description='Start the measured-state robot_state_publisher.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz; false is useful for headless tests.',
        ),
        physical_state_publisher,
        preview_state_publisher,
        preview_anchor,
        rviz,
    ])
