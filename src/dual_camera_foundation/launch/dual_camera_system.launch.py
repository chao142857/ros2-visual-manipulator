"""Compatibility entry point for the canonical system bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Forward the former system arguments without duplicating owners."""
    arguments = {
        'start_robot': 'false',
        'start_d405': 'false',
        'start_astra': 'false',
        'start_perception': 'false',
        'start_console': 'false',
        'start_arm_driver': 'false',
        'start_gripper_driver': 'false',
        'd405_serial': '1',
        'astra_serial': '',
        'd405_align_depth': 'false',
        'perception_input_topic': '/camera/tag_point',
    }
    declared = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in arguments.items()
    ]
    declared.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dual_camera_foundation'),
            'launch',
            'system_bringup.launch.py',
        ])),
        launch_arguments={
            'start_robot_state': LaunchConfiguration('start_robot'),
            'start_d405': LaunchConfiguration('start_d405'),
            'start_astra': LaunchConfiguration('start_astra'),
            'start_perception': LaunchConfiguration('start_perception'),
            'start_console': LaunchConfiguration('start_console'),
            'start_arm_driver': LaunchConfiguration('start_arm_driver'),
            'start_gripper_driver': LaunchConfiguration(
                'start_gripper_driver'
            ),
            'd405_serial': LaunchConfiguration('d405_serial'),
            'astra_serial': LaunchConfiguration('astra_serial'),
            'd405_align_depth': LaunchConfiguration('d405_align_depth'),
            'perception_astra_input_topic': LaunchConfiguration(
                'perception_input_topic'
            ),
        }.items(),
    ))
    return LaunchDescription(declared)
