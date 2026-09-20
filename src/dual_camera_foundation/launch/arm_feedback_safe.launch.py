"""Mock-first arm feedback entry point; START/MOVE are always disabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build a mock-by-default stage with motion commands disabled."""
    system_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dual_camera_foundation'),
            'launch',
            'system_bringup.launch.py',
        ])),
        launch_arguments={
            'start_robot_state': 'true',
            'start_arm_driver': 'true',
            'arm_use_mock': LaunchConfiguration('arm_use_mock'),
            'arm_allow_motion_commands': 'false',
            'arm_config_file': LaunchConfiguration('arm_config_file'),
            'arm_serial_port': LaunchConfiguration('arm_serial_port'),
            'arm_baud_rate': LaunchConfiguration('arm_baud_rate'),
            'arm_poll_period': LaunchConfiguration('arm_poll_period'),
            'start_gripper_driver': 'false',
            'start_astra': 'false',
            'start_d405': 'false',
            'start_perception': 'false',
            'start_console': 'false',
            'start_capture': 'false',
            'start_dry_run': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_use_mock',
            default_value='true',
            description=(
                'Keep true for the first stage. Set false only to read the '
                'real controller; START/MOVE remain disabled. STOP/DISABLE services '
                'can still write to hardware in real feedback mode.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('arm_console'),
                'config',
                'arm_console.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'arm_serial_port',
            default_value='/dev/ttyACM0',
        ),
        DeclareLaunchArgument('arm_baud_rate', default_value='115200'),
        DeclareLaunchArgument('arm_poll_period', default_value='0.1'),
        system_bringup,
    ])
