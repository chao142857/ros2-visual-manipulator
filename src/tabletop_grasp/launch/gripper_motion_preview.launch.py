"""Inspect the detached approximate gripper; this launch has no robot interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory('tabletop_grasp'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pulse_us', default_value='1300'),
        DeclareLaunchArgument('show_bounds', default_value='true'),
        Node(package='tabletop_grasp', executable='gripper_motion_preview', output='screen',
             parameters=[{
                 'pulse_us': ParameterValue(LaunchConfiguration('pulse_us'), value_type=float),
                 'show_bounds': ParameterValue(LaunchConfiguration('show_bounds'),
                                               value_type=bool),
             }]),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(package / 'config/gripper_motion_preview.rviz')]),
    ])
