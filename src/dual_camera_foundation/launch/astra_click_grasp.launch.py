"""Single-terminal Astra click-to-grasp IK preview workflow.

The launch owns one Astra and one arm-state driver. It never starts Arm
Console, D405, or the gripper, and visual grasp is hard-wired to dry-run.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _choice(context, name, allowed):
    """Resolve and validate one enumerated launch argument."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in allowed:
        choices = ', '.join(sorted(allowed))
        raise ValueError(f'{name} must be one of [{choices}], got {value!r}')
    return value


def _as_bool(context, name):
    """Resolve one launch boolean without accepting misspellings."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got {value!r}')


def _validator_exit_actions(event, context):
    """Propagate validator failure to the ros2 launch process."""
    del context
    if event.returncode == 0:
        return [EmitEvent(event=Shutdown(
            reason='Astra click-grasp validation passed',
        ))]
    return [OpaqueFunction(
        function=_raise_validator_failure,
        args=[event.returncode],
    )]


def _raise_validator_failure(context, returncode):
    """Raise inside LaunchService so Humble returns a nonzero status."""
    del context
    raise RuntimeError(
        'Astra click-grasp validation failed with return code '
        f'{returncode}'
    )


def _launch_setup(context):
    """Create the single-owner, preview-only workflow actions."""
    arm_mode = _choice(context, 'arm_mode', {'mock', 'real'})
    validation_level = _choice(
        context, 'validation_level', {'startup', 'click', 'ik'}
    )
    shutdown_after_validation = _as_bool(
        context, 'shutdown_after_validation'
    )

    system_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dual_camera_foundation'),
            'launch',
            'system_bringup.launch.py',
        ])),
        launch_arguments={
            'start_robot_state': 'true',
            'start_arm_driver': 'true',
            'arm_use_mock': 'true' if arm_mode == 'mock' else 'false',
            'arm_allow_motion_commands': 'false',
            'arm_config_file': LaunchConfiguration('arm_config_file'),
            'arm_serial_port': LaunchConfiguration('arm_serial_port'),
            'arm_baud_rate': LaunchConfiguration('arm_baud_rate'),
            'arm_poll_period': LaunchConfiguration('arm_poll_period'),
            'start_gripper_driver': 'false',
            'start_astra': 'true',
            'astra_serial': LaunchConfiguration('astra_serial'),
            'astra_camera_name': 'camera',
            'start_d405': 'false',
            'start_perception': 'false',
            'start_astra_perception': 'false',
            'start_d405_perception': 'false',
            'start_refiner': 'false',
            'start_console': 'false',
            'start_capture': 'false',
            'start_dry_run': 'false',
            'use_mount_calibration': 'false',
        }.items(),
    )

    click_target = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('camera_perception'),
            'launch',
            'astra_click_target.launch.py',
        ])),
        launch_arguments={
            'extrinsics_file': LaunchConfiguration('extrinsics_file'),
            'enable_optical_flow_tracking': 'false',
        }.items(),
    )

    visual_grasp = Node(
        package='dual_camera_foundation',
        executable='visual_grasp',
        name='visual_grasp_controller',
        output='screen',
        parameters=[{
            'execute_hardware': False,
            'workspace_bounds_confirmed': False,
        }],
    )

    validator = Node(
        package='dual_camera_foundation',
        executable='astra_click_grasp_validator',
        name='astra_click_grasp_validator',
        output='screen',
        parameters=[{
            'validation_level': validation_level,
            'startup_timeout_sec': float(
                LaunchConfiguration('startup_timeout_sec').perform(context)
            ),
            'click_timeout_sec': float(
                LaunchConfiguration('click_timeout_sec').perform(context)
            ),
            'ik_timeout_sec': float(
                LaunchConfiguration('ik_timeout_sec').perform(context)
            ),
            'require_zero_command_publishers': True,
        }],
    )

    actions = [system_bringup, click_target, visual_grasp, validator]
    if shutdown_after_validation:
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=validator,
            on_exit=_validator_exit_actions,
        )))
    return actions


def generate_launch_description():
    """Declare the deliberately small public interface for this workflow."""
    default_extrinsics = PathJoinSubstitution([
        FindPackageShare('camera_perception'),
        'config',
        'camera_extrinsics.yaml',
    ])
    default_arm_config = PathJoinSubstitution([
        FindPackageShare('arm_console'),
        'config',
        'arm_console.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_mode',
            default_value='mock',
            description=(
                'mock publishes simulated feedback; real opens the existing '
                'arm feedback driver but still never sends grasp commands.'
            ),
        ),
        DeclareLaunchArgument(
            'validation_level',
            default_value='ik',
            description='Stop after startup, click-chain, or full IK checks.',
        ),
        DeclareLaunchArgument(
            'shutdown_after_validation',
            default_value='true',
            description=(
                'Shut down the complete launch after the validator exits. '
                'Set false to keep the dry-run preview open after validation.'
            ),
        ),
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=default_extrinsics,
        ),
        DeclareLaunchArgument('astra_serial', default_value=''),
        DeclareLaunchArgument(
            'arm_config_file',
            default_value=default_arm_config,
        ),
        DeclareLaunchArgument(
            'arm_serial_port',
            default_value='/dev/ttyACM0',
        ),
        DeclareLaunchArgument('arm_baud_rate', default_value='115200'),
        DeclareLaunchArgument('arm_poll_period', default_value='0.1'),
        DeclareLaunchArgument('startup_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('click_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('ik_timeout_sec', default_value='20.0'),
        OpaqueFunction(function=_launch_setup),
    ])
