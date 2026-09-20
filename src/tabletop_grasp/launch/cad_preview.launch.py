"""Inspect static CAD and conservative proxy, detached from every robot frame."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('tabletop_grasp'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='tabletop_grasp', executable='cad_preview', output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(package / 'config/cad_preview.rviz')]),
    ])
