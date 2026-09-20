"""Compatibility entry point for the canonical dual-camera bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Forward legacy foundation switches to the single-owner bringup."""
    return LaunchDescription([
        DeclareLaunchArgument('start_robot', default_value='false'),
        DeclareLaunchArgument('start_astra', default_value='false'),
        DeclareLaunchArgument('start_d405', default_value='false'),
        DeclareLaunchArgument('start_perception', default_value='false'),
        DeclareLaunchArgument('start_refiner', default_value='false'),
        DeclareLaunchArgument('start_dry_run', default_value='false'),
        DeclareLaunchArgument('d405_serial', default_value='1'),
        DeclareLaunchArgument('d405_align_depth', default_value='false'),
        DeclareLaunchArgument('astra_serial', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('dual_camera_foundation'),
                'launch',
                'system_bringup.launch.py',
            ])),
            launch_arguments={
                'start_robot_state': LaunchConfiguration('start_robot'),
                'start_astra': LaunchConfiguration('start_astra'),
                'start_d405': LaunchConfiguration('start_d405'),
                'start_perception': LaunchConfiguration('start_perception'),
                'start_refiner': LaunchConfiguration('start_refiner'),
                'start_dry_run': LaunchConfiguration('start_dry_run'),
                'd405_serial': LaunchConfiguration('d405_serial'),
                'd405_align_depth': LaunchConfiguration(
                    'd405_align_depth'
                ),
                'astra_serial': LaunchConfiguration('astra_serial'),
            }.items(),
        ),
    ])
