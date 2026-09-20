"""Canonical single-owner system bringup.

Every physical resource is opt-in. This file owns at most one Astra driver and
one D405 driver; the Console is GUI-only and never starts serial drivers. The
deprecated ``start_robot`` and ``start_cameras`` arguments are compatibility
aliases and are folded into the primary switches once.
"""

from dual_camera_foundation.mount_tf_publisher import validate_mount_contract
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value, name):
    """Parse a launch boolean and reject silently accepted typos."""
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off', ''):
        return False
    raise ValueError(f'{name} must be true or false, got {value!r}')


def _include(package, file_name, arguments):
    """Create a project/vendor launch include without a condition."""
    source_type = (
        AnyLaunchDescriptionSource
        if file_name.endswith('.xml')
        else PythonLaunchDescriptionSource
    )
    return IncludeLaunchDescription(
        source_type(PathJoinSubstitution([
            FindPackageShare(package), 'launch', file_name,
        ])),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context):
    """Resolve aliases and create exactly one action for each owner."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    start_robot_state = _as_bool(
        value('start_robot_state'), 'start_robot_state'
    )
    legacy_robot = value('start_robot').strip()
    if legacy_robot:
        start_robot_state = _as_bool(legacy_robot, 'start_robot')

    start_astra = _as_bool(value('start_astra'), 'start_astra')
    start_d405 = _as_bool(value('start_d405'), 'start_d405')
    legacy_cameras = value('start_cameras').strip()
    if legacy_cameras and _as_bool(legacy_cameras, 'start_cameras'):
        start_astra = True
        start_d405 = True

    start_arm = _as_bool(value('start_arm_driver'), 'start_arm_driver')
    start_gripper = _as_bool(
        value('start_gripper_driver'), 'start_gripper_driver'
    )
    start_perception = _as_bool(
        value('start_perception'), 'start_perception'
    )
    start_astra_perception = _as_bool(
        value('start_astra_perception'), 'start_astra_perception'
    ) or start_perception
    start_astra_transform = _as_bool(
        value('start_astra_transform'), 'start_astra_transform'
    )
    start_d405_perception = _as_bool(
        value('start_d405_perception'), 'start_d405_perception'
    ) or start_perception
    start_refiner = _as_bool(
        value('start_refiner'), 'start_refiner'
    ) or start_perception
    start_console = _as_bool(value('start_console'), 'start_console')
    start_capture = _as_bool(value('start_capture'), 'start_capture')
    start_dry_run = _as_bool(value('start_dry_run'), 'start_dry_run')

    d405_align_depth = _as_bool(
        value('d405_align_depth'), 'd405_align_depth'
    )
    if start_d405_perception and not d405_align_depth:
        raise ValueError(
            'start_d405_perception requires d405_align_depth:=true '
            'because the default D405 chain consumes aligned depth'
        )
    if start_capture and not start_robot_state:
        raise ValueError('start_capture requires start_robot_state:=true')

    use_mount_calibration = _as_bool(
        value('use_mount_calibration'), 'use_mount_calibration'
    )
    start_d405_transform = _as_bool(
        value('start_d405_transform'), 'start_d405_transform'
    )
    if start_d405_transform and not start_d405_perception:
        raise ValueError(
            'start_d405_transform:=true requires '
            'start_d405_perception:=true'
        )

    camera_namespace = value('d405_camera_namespace').strip().strip('/')
    camera_name = value('d405_camera_name').strip()
    if not camera_namespace or not camera_name:
        raise ValueError('D405 camera namespace and name must be non-empty')
    d405_topic_prefix = f'/{camera_namespace}/{camera_name}'
    d405_optical_frame = f'{camera_name}_color_optical_frame'

    mount_file = value('d405_mount_calibration')
    mount_parent = value('d405_mount_parent_frame')
    mount_child = value('d405_mount_child_frame')
    mount_minimum_samples = int(value('mount_minimum_sample_count'))
    mount_max_translation = float(value('mount_max_translation_rms_m'))
    mount_max_rotation = float(value('mount_max_rotation_rms_deg'))
    mount_max_outliers = int(value('mount_max_outlier_count'))
    validate_mount_contract(
        start_d405_transform,
        use_mount_calibration,
        mount_file,
        mount_parent,
        mount_child,
        f'{camera_name}_link',
        mount_minimum_samples,
        mount_max_translation,
        mount_max_rotation,
        mount_max_outliers,
    )

    actions = []
    if start_robot_state:
        actions.append(_include(
            'arm_description', 'robot_state.launch.py', {}
        ))

    arm_config = value('arm_config_file')
    if start_arm:
        actions.append(Node(
            package='arm_driver',
            executable='arm_driver_node',
            name='arm_driver_node',
            output='screen',
            parameters=[arm_config, {
                'use_mock': _as_bool(value('arm_use_mock'), 'arm_use_mock'),
                'allow_motion_commands': _as_bool(
                    value('arm_allow_motion_commands'),
                    'arm_allow_motion_commands',
                ),
                'serial_port': value('arm_serial_port'),
                'baud_rate': int(value('arm_baud_rate')),
                'poll_period': float(value('arm_poll_period')),
            }],
        ))

    if start_gripper:
        actions.append(Node(
            package='gripper_driver',
            executable='gripper_driver_node',
            name='gripper_driver_node',
            output='screen',
            parameters=[{
                'allow_hardware_actuation': _as_bool(
                    value('gripper_allow_hardware_actuation'),
                    'gripper_allow_hardware_actuation',
                ),
                'serial_port': value('gripper_serial_port'),
                'baud_rate': int(value('gripper_baud_rate')),
                'gripper_channel': int(value('gripper_channel')),
                'open_pulse_us': int(value('gripper_open_pulse_us')),
                'close_pulse_us': int(value('gripper_close_pulse_us')),
                'move_duration_ms': int(value('gripper_move_duration_ms')),
            }],
        ))

    if start_astra:
        actions.append(_include('astra_camera', 'astra_pro.launch.xml', {
            'camera_name': value('astra_camera_name'),
            'serial_number': value('astra_serial'),
            'depth_registration': 'true',
            'color_depth_synchronization': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'depth_scale': '1',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'tf_publish_rate': '0.0',
        }))

    if start_d405:
        actions.append(_include(
            'wrist_camera_bringup',
            'wrist_d405.launch.py',
            {
                'start_camera': 'true',
                'start_health_monitor': value('start_d405_health_monitor'),
                'camera_namespace': value('d405_camera_namespace'),
                'camera_name': value('d405_camera_name'),
                'serial_no': value('d405_serial'),
                'align_depth': value('d405_align_depth'),
                'pointcloud': value('d405_pointcloud'),
                'publish_uncalibrated_mount_tf': 'false',
                'require_mount_tf': str(use_mount_calibration).lower(),
            },
        ))

    if use_mount_calibration:
        actions.append(Node(
            package='dual_camera_foundation',
            executable='calibrated_mount_tf_publisher',
            name='calibrated_wrist_camera_mount_publisher',
            output='screen',
            parameters=[{
                'calibration_file': mount_file,
                'parent_frame': mount_parent,
                'child_frame': mount_child,
                'minimum_sample_count': mount_minimum_samples,
                'max_translation_rms_m': mount_max_translation,
                'max_rotation_rms_deg': mount_max_rotation,
                'max_outlier_count': mount_max_outliers,
            }],
        ))

    if start_astra_perception:
        actions.append(_include(
            'camera_perception',
            'camera_runtime.launch.py',
            {
                'target_input_topic': value('perception_astra_input_topic'),
                'target_output_topic': value('perception_astra_output_topic'),
                'target_frame': value('perception_target_frame'),
                'publish_marker': value('perception_publish_marker'),
                'start_transform': str(start_astra_transform).lower(),
                'start_tag_localizer': 'true',
            },
        ))

    if start_d405_perception:
        actions.append(_include(
            'dual_camera_foundation',
            'd405_rgbd_perception.launch.py',
            {
                'topic_prefix': d405_topic_prefix,
                'target_pose_topic': (
                    f'{d405_topic_prefix}/calibration/target_pose'
                ),
                'base_pose_topic': '/perception/d405_target_pose_base',
                'target_pixel_topic': (
                    f'{d405_topic_prefix}/calibration/target_pixel'
                ),
                'depth_topic': (
                    f'{d405_topic_prefix}/'
                    'aligned_depth_to_color/image_raw'
                ),
                'depth_info_topic': (
                    f'{d405_topic_prefix}/'
                    'aligned_depth_to_color/camera_info'
                ),
                'point_topic': value('perception_d405_input_topic'),
                'base_point_topic': value('perception_d405_output_topic'),
                'start_transform': str(start_d405_transform).lower(),
                'target_frame': value('perception_target_frame'),
                'calibration_target_frame': value(
                    'd405_calibration_target_frame'
                ),
                'publish_tf': value('d405_perception_publish_tf'),
                'publish_marker': value('perception_publish_marker'),
                'target_tag_id': value('d405_target_tag_id'),
                'max_depth_age_sec': value('d405_max_depth_age_sec'),
                'future_tolerance_sec': value(
                    'd405_depth_future_tolerance_sec'
                ),
                'depth_queue_size': value('d405_depth_queue_size'),
            },
        ))

    if start_refiner:
        actions.append(Node(
            package='dual_camera_foundation',
            executable='dual_camera_refiner',
            name='dual_camera_refiner',
            output='screen',
            parameters=[{
                'astra_topic': value('perception_astra_output_topic'),
                'd405_topic': value('perception_d405_output_topic'),
                'output_topic': value('perception_dual_output_topic'),
                'astra_max_age_sec': float(
                    value('refiner_astra_max_age_sec')
                ),
                'd405_max_age_sec': float(
                    value('refiner_d405_max_age_sec')
                ),
                'future_tolerance_sec': float(
                    value('refiner_future_tolerance_sec')
                ),
                'max_pair_time_delta_sec': float(
                    value('refiner_max_pair_time_delta_sec')
                ),
                'max_pair_distance_m': float(
                    value('refiner_max_pair_distance_m')
                ),
                'max_output_jump_m': float(
                    value('refiner_max_output_jump_m')
                ),
                'output_jump_reference_max_age_sec': float(
                    value('refiner_output_jump_reference_max_age_sec')
                ),
                'require_astra_reference': _as_bool(
                    value('refiner_require_astra_reference'),
                    'refiner_require_astra_reference',
                ),
                'require_same_frame': _as_bool(
                    value('refiner_require_same_frame'),
                    'refiner_require_same_frame',
                ),
                'allow_astra_fallback': _as_bool(
                    value('allow_astra_fallback'),
                    'allow_astra_fallback',
                ),
            }],
        ))

    if start_console:
        actions.append(_include(
            'arm_console',
            'arm_console.launch.py',
            {
                'start_driver': 'false',
                'start_gripper_driver': 'false',
                'gripper_open_pulse_us': value('gripper_open_pulse_us'),
                'gripper_close_pulse_us': value('gripper_close_pulse_us'),
            },
        ))

    if start_capture:
        actions.append(_include(
            'dual_camera_foundation',
            'hand_eye_capture_canonical.launch.py',
            {
                'base_frame': value('hand_eye_base_frame'),
                'tool_frame': mount_parent,
                'camera_frame': d405_optical_frame,
                'target_frame': value('d405_calibration_target_frame'),
                'target_pose_topic': (
                    f'{d405_topic_prefix}/calibration/target_pose'
                ),
                'output_file': value('hand_eye_output_file'),
                'sample_count': value('hand_eye_sample_count'),
                'overwrite': value('hand_eye_overwrite'),
            },
        ))

    if start_dry_run:
        actions.append(Node(
            package='dual_camera_foundation',
            executable='dry_run_grasp_targets',
            name='dry_run_grasp_targets',
            output='screen',
            parameters=[{
                'input_topic': value('perception_dual_output_topic'),
            }],
        ))
    return actions


def generate_launch_description():
    """Declare primary switches and compatibility aliases."""
    return LaunchDescription([
        DeclareLaunchArgument('start_robot_state', default_value='true'),
        DeclareLaunchArgument('start_arm_driver', default_value='false'),
        DeclareLaunchArgument('start_gripper_driver', default_value='false'),
        DeclareLaunchArgument(
            'gripper_allow_hardware_actuation', default_value='false',
            description='Explicit opt-in to open gripper serial and accept PWM commands.',
        ),
        DeclareLaunchArgument('start_astra', default_value='false'),
        DeclareLaunchArgument('start_d405', default_value='false'),
        DeclareLaunchArgument('start_perception', default_value='false'),
        DeclareLaunchArgument(
            'start_astra_perception', default_value='false'
        ),
        DeclareLaunchArgument(
            'start_d405_perception', default_value='false'
        ),
        DeclareLaunchArgument('start_refiner', default_value='false'),
        DeclareLaunchArgument('start_console', default_value='false'),
        DeclareLaunchArgument('start_capture', default_value='false'),
        DeclareLaunchArgument('start_dry_run', default_value='false'),
        DeclareLaunchArgument(
            'use_mount_calibration', default_value='false'
        ),
        DeclareLaunchArgument('d405_mount_calibration', default_value=''),
        DeclareLaunchArgument(
            'd405_mount_parent_frame', default_value='tool0'
        ),
        DeclareLaunchArgument(
            'd405_mount_child_frame', default_value='wrist_d405_link'
        ),
        DeclareLaunchArgument(
            'mount_minimum_sample_count', default_value='15'
        ),
        DeclareLaunchArgument(
            'mount_max_translation_rms_m', default_value='0.01'
        ),
        DeclareLaunchArgument(
            'mount_max_rotation_rms_deg', default_value='2.0'
        ),
        DeclareLaunchArgument(
            'mount_max_outlier_count', default_value='0'
        ),
        DeclareLaunchArgument('start_robot', default_value=''),
        DeclareLaunchArgument('start_cameras', default_value=''),
        DeclareLaunchArgument(
            'arm_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('arm_console'),
                'config',
                'arm_console.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'arm_use_mock',
            default_value='true',
            description=(
                'Safe default: do not open the real arm serial port unless '
                'arm_use_mock:=false is explicitly selected.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_allow_motion_commands',
            default_value='false',
            description=(
                'Arm motion authority passed to arm_driver_node.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_serial_port', default_value='/dev/ttyACM0'
        ),
        DeclareLaunchArgument('arm_baud_rate', default_value='115200'),
        DeclareLaunchArgument('arm_poll_period', default_value='0.1'),
        DeclareLaunchArgument(
            'gripper_serial_port',
            default_value=(
                '/dev/serial/by-id/CONFIGURE_GRIPPER_PORT'
            ),
        ),
        DeclareLaunchArgument('gripper_baud_rate', default_value='115200'),
        DeclareLaunchArgument('gripper_channel', default_value='0'),
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
            'gripper_move_duration_ms', default_value='600'
        ),
        DeclareLaunchArgument(
            'd405_serial', default_value='1'
        ),
        DeclareLaunchArgument('d405_align_depth', default_value='false'),
        DeclareLaunchArgument('d405_pointcloud', default_value='false'),
        DeclareLaunchArgument(
            'start_d405_health_monitor', default_value='true'
        ),
        DeclareLaunchArgument(
            'd405_camera_namespace', default_value='wrist_camera'
        ),
        DeclareLaunchArgument(
            'd405_camera_name', default_value='wrist_d405'
        ),
        DeclareLaunchArgument('astra_serial', default_value=''),
        DeclareLaunchArgument(
            'astra_camera_name', default_value='camera'
        ),
        DeclareLaunchArgument(
            'start_astra_transform', default_value='true'
        ),
        DeclareLaunchArgument(
            'start_d405_transform', default_value='false'
        ),
        DeclareLaunchArgument(
            'd405_max_depth_age_sec', default_value='0.05'
        ),
        DeclareLaunchArgument(
            'd405_depth_future_tolerance_sec', default_value='0.05'
        ),
        DeclareLaunchArgument(
            'd405_depth_queue_size', default_value='8'
        ),
        DeclareLaunchArgument(
            'perception_target_frame', default_value='base_link'
        ),
        DeclareLaunchArgument(
            'perception_publish_marker', default_value='false'
        ),
        DeclareLaunchArgument(
            'd405_perception_publish_tf', default_value='true'
        ),
        DeclareLaunchArgument('d405_target_tag_id', default_value='6'),
        DeclareLaunchArgument(
            'd405_calibration_target_frame',
            default_value='wrist_calibration_target_6',
        ),
        DeclareLaunchArgument(
            'hand_eye_base_frame', default_value='base_link'
        ),
        DeclareLaunchArgument(
            'hand_eye_output_file',
            default_value='/tmp/d405_hand_eye_samples.yaml',
        ),
        DeclareLaunchArgument(
            'hand_eye_sample_count', default_value='20'
        ),
        DeclareLaunchArgument(
            'hand_eye_overwrite', default_value='false'
        ),
        DeclareLaunchArgument(
            'refiner_astra_max_age_sec', default_value='1.0'
        ),
        DeclareLaunchArgument(
            'refiner_d405_max_age_sec', default_value='0.35'
        ),
        DeclareLaunchArgument(
            'refiner_future_tolerance_sec', default_value='0.05'
        ),
        DeclareLaunchArgument(
            'refiner_max_pair_time_delta_sec', default_value='0.25'
        ),
        DeclareLaunchArgument(
            'refiner_max_pair_distance_m', default_value='0.08'
        ),
        DeclareLaunchArgument(
            'refiner_max_output_jump_m', default_value='0.08'
        ),
        DeclareLaunchArgument(
            'refiner_output_jump_reference_max_age_sec',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'refiner_require_astra_reference', default_value='true'
        ),
        DeclareLaunchArgument(
            'refiner_require_same_frame', default_value='true'
        ),
        DeclareLaunchArgument(
            'allow_astra_fallback', default_value='true'
        ),
        DeclareLaunchArgument(
            'perception_astra_input_topic',
            default_value='/camera/tag_point',
        ),
        DeclareLaunchArgument(
            'perception_astra_output_topic',
            default_value='/perception/astra_target_point_base',
        ),
        DeclareLaunchArgument(
            'perception_d405_input_topic',
            default_value='/perception/d405_target_point_camera',
        ),
        DeclareLaunchArgument(
            'perception_d405_output_topic',
            default_value='/perception/d405_target_point_base',
        ),
        DeclareLaunchArgument(
            'perception_dual_output_topic',
            default_value='/perception/dual_target_point_base',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
