"""Launch Arm Console with an optionally launch-owned Astra Pro driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose the vendor driver and Console without GUI process ownership."""
    start_camera = LaunchConfiguration('start_camera')
    camera_serial_number = LaunchConfiguration('camera_serial_number')
    camera_enable_ir = LaunchConfiguration('camera_enable_ir')
    camera_enable_point_cloud = LaunchConfiguration(
        'camera_enable_point_cloud'
    )
    camera_enable_colored_point_cloud = LaunchConfiguration(
        'camera_enable_colored_point_cloud'
    )
    camera_tf_publish_rate = LaunchConfiguration('camera_tf_publish_rate')

    start_driver = LaunchConfiguration('start_driver')
    use_mock = LaunchConfiguration('use_mock')
    allow_motion_commands = LaunchConfiguration('arm_allow_motion_commands')
    start_gripper_driver = LaunchConfiguration('start_gripper_driver')
    gripper_open_pulse_us = LaunchConfiguration('gripper_open_pulse_us')
    gripper_close_pulse_us = LaunchConfiguration('gripper_close_pulse_us')

    astra = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch',
                'astra_pro.launch.xml',
            ])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            'camera_name': 'camera',
            'serial_number': camera_serial_number,
            'depth_registration': 'true',
            'color_depth_synchronization': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'depth_scale': '1',
            'enable_ir': camera_enable_ir,
            'enable_point_cloud': camera_enable_point_cloud,
            'enable_colored_point_cloud': (
                camera_enable_colored_point_cloud
            ),
            'tf_publish_rate': camera_tf_publish_rate,
            'oni_log_level': 'warning',
            'oni_log_to_console': 'false',
            'oni_log_to_file': 'false',
        }.items(),
    )

    console = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_console'),
                'launch',
                'arm_console.launch.py',
            ])
        ),
        launch_arguments={
            'start_driver': start_driver,
            'use_mock': use_mock,
            'arm_allow_motion_commands': allow_motion_commands,
            'gripper_allow_hardware_actuation': LaunchConfiguration(
                'gripper_allow_hardware_actuation'
            ),
            'start_gripper_driver': start_gripper_driver,
            'gripper_open_pulse_us': gripper_open_pulse_us,
            'gripper_close_pulse_us': gripper_close_pulse_us,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description=(
                'Start an Astra Pro process owned by this launch. Set false '
                'when a camera driver already runs externally.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_serial_number',
            default_value='',
            description='Optional Astra serial number.',
        ),
        DeclareLaunchArgument(
            'camera_enable_ir',
            default_value='false',
            description='Enable the unused IR stream.',
        ),
        DeclareLaunchArgument(
            'camera_enable_point_cloud',
            default_value='false',
            description='Enable uncoloured point-cloud conversion.',
        ),
        DeclareLaunchArgument(
            'camera_enable_colored_point_cloud',
            default_value='false',
            description='Enable registered RGB point-cloud conversion.',
        ),
        DeclareLaunchArgument(
            'camera_tf_publish_rate',
            default_value='0.0',
            description=(
                'Zero publishes fixed internal camera transforms on '
                '/tf_static.'
            ),
        ),
        DeclareLaunchArgument(
            'start_driver',
            default_value='false',
            description='Pass through to arm_console.launch.py.',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='true',
            description='Pass through to arm_console.launch.py.',
        ),
        DeclareLaunchArgument(
            'arm_allow_motion_commands',
            default_value='false',
            description=(
                'Arm motion authority passed through to the optional driver.'
            ),
        ),
        DeclareLaunchArgument(
            'gripper_allow_hardware_actuation', default_value='false',
            description='Explicit opt-in to open gripper serial and accept PWM commands.',
        ),
        DeclareLaunchArgument(
            'start_gripper_driver',
            default_value='false',
            description='Pass through to arm_console.launch.py.',
        ),
        DeclareLaunchArgument(
            'gripper_open_pulse_us',
            default_value='600',
            description='Measured fully-open gripper pulse.',
        ),
        DeclareLaunchArgument(
            'gripper_close_pulse_us',
            default_value='1300',
            description='Measured object-gripping pulse.',
        ),
        astra,
        console,
    ])
