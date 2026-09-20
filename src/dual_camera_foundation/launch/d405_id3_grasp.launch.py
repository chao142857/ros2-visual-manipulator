"""Deprecated name redirected to the tag-free, planning-only tabletop stack."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='d405_id3_grasp is deprecated. Starting tabletop_grasp: '
                    'no runtime tag, no arm/gripper driver, no hardware execution.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('tabletop_grasp'), 'launch', 'tabletop_grasp.launch.py',
        ]))),
    ])
