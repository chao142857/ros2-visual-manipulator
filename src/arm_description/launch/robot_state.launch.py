"""Publish the Dummy V2 robot state without starting RViz or hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the project robot-state publisher."""
    package_share = get_package_share_directory('arm_description')
    urdf_path = os.path.join(package_share, 'urdf', 'dummy-v2.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the ROS simulation clock.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
            }],
        ),
    ])
