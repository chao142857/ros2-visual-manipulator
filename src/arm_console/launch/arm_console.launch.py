"""Start the desktop console, optionally with the existing arm driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the real-or-mock arm console launch description."""
    package_share = get_package_share_directory('arm_console')
    config_path = os.path.join(package_share, 'config', 'arm_console.yaml')

    start_driver = LaunchConfiguration('start_driver')
    use_mock = LaunchConfiguration('use_mock')
    allow_motion_commands = LaunchConfiguration('arm_allow_motion_commands')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    poll_period = LaunchConfiguration('poll_period')
    start_gripper_driver = LaunchConfiguration('start_gripper_driver')
    gripper_serial_port = LaunchConfiguration('gripper_serial_port')
    gripper_baud_rate = LaunchConfiguration('gripper_baud_rate')
    gripper_channel = LaunchConfiguration('gripper_channel')
    gripper_open_pulse_us = LaunchConfiguration('gripper_open_pulse_us')
    gripper_close_pulse_us = LaunchConfiguration('gripper_close_pulse_us')
    gripper_move_duration_ms = LaunchConfiguration(
        'gripper_move_duration_ms'
    )

    driver = Node(
        package='arm_driver',
        executable='arm_driver_node',
        name='arm_driver_node',
        output='screen',
        condition=IfCondition(start_driver),
        parameters=[
            config_path,
            {
                'use_mock': ParameterValue(use_mock, value_type=bool),
                'allow_motion_commands': ParameterValue(
                    allow_motion_commands,
                    value_type=bool,
                ),
                'serial_port': serial_port,
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'poll_period': ParameterValue(poll_period, value_type=float),
            },
        ],
    )

    gripper_driver = Node(
        package='gripper_driver',
        executable='gripper_driver_node',
        name='gripper_driver_node',
        output='screen',
        condition=IfCondition(start_gripper_driver),
        parameters=[{
            'allow_hardware_actuation': ParameterValue(
                LaunchConfiguration('gripper_allow_hardware_actuation'),
                value_type=bool,
            ),
            'serial_port': gripper_serial_port,
            'baud_rate': ParameterValue(gripper_baud_rate, value_type=int),
            'gripper_channel': ParameterValue(gripper_channel, value_type=int),
            'open_pulse_us': ParameterValue(
                gripper_open_pulse_us,
                value_type=int,
            ),
            'close_pulse_us': ParameterValue(
                gripper_close_pulse_us,
                value_type=int,
            ),
            'move_duration_ms': ParameterValue(
                gripper_move_duration_ms,
                value_type=int,
            ),
        }],
    )

    console = Node(
        package='arm_console',
        executable='arm_console',
        name='arm_console',
        output='screen',
        parameters=[
            config_path,
            {
                'gripper_open_pulse_us': ParameterValue(
                    gripper_open_pulse_us,
                    value_type=int,
                ),
                'gripper_close_pulse_us': ParameterValue(
                    gripper_close_pulse_us,
                    value_type=int,
                ),
            },
        ],
    )

    shutdown_when_console_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=console,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='Arm Console closed')
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_driver',
            default_value='false',
            description='Start arm_driver_node from this launch file.',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='true',
            description='Run arm_driver_node without opening the serial port.',
        ),
        DeclareLaunchArgument(
            'arm_allow_motion_commands',
            default_value='false',
            description=(
                'Arm motion authority passed to arm_driver_node.'
            ),
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port passed only to arm_driver_node.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate passed only to arm_driver_node.',
        ),
        DeclareLaunchArgument(
            'poll_period',
            default_value='0.1',
            description='Joint state polling period in seconds.',
        ),
        DeclareLaunchArgument(
            'gripper_allow_hardware_actuation', default_value='false',
            description='Explicit opt-in to open gripper serial and accept PWM commands.',
        ),
        DeclareLaunchArgument(
            'start_gripper_driver',
            default_value='false',
            description='Start the existing gripper_driver_node.',
        ),
        DeclareLaunchArgument(
            'gripper_serial_port',
            default_value=(
                '/dev/serial/by-id/CONFIGURE_GRIPPER_PORT'
            ),
            description='Serial port passed only to gripper_driver_node.',
        ),
        DeclareLaunchArgument(
            'gripper_baud_rate',
            default_value='115200',
            description='Gripper controller baud rate.',
        ),
        DeclareLaunchArgument(
            'gripper_channel',
            default_value='0',
            description='PCA9685 channel used by the gripper servo.',
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
        DeclareLaunchArgument(
            'gripper_move_duration_ms',
            default_value='600',
            description='Gripper controller move duration.',
        ),
        driver,
        gripper_driver,
        console,
        shutdown_when_console_closes,
    ])
