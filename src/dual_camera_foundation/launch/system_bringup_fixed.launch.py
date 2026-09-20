"""Compatibility alias for the corrected canonical system bringup."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Include the canonical launch; all hardware remains opt-in there."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('dual_camera_foundation'),
                'launch',
                'system_bringup.launch.py',
            ])),
        ),
    ])
