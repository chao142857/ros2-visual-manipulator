"""Launch the Dummy V2 robot-state publisher and RViz model view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the offline Dummy V2 visualization launch description."""
    package_share = get_package_share_directory(
        'arm_description'
    )

    urdf_path = os.path.join(
        package_share,
        'urdf',
        'dummy-v2.urdf'
    )

    rviz_config_path = os.path.join(
        package_share,
        'rviz',
        'arm.rviz'
    )

    with open(urdf_path, 'r', encoding='utf-8') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config_path,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
    ])
