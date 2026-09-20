"""Run guarded Astra/D405 transforms and the dual-camera selector.

Astra may use latest TF because it is fixed. D405 is mounted on the moving
wrist and uses each observation stamp. Enabling the D405 base transform also
loads and validates the mount calibration in this launch; invalid or missing
calibration fails before any D405 base-frame point can be published.
"""

from dual_camera_foundation.mount_tf_publisher import validate_mount_contract
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false')


def _setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    start_astra = _as_bool(
        value('start_astra_transform'),
        'start_astra_transform',
    )
    start_d405 = _as_bool(
        value('start_d405_transform'),
        'start_d405_transform',
    )
    use_mount = _as_bool(
        value('use_mount_calibration'),
        'use_mount_calibration',
    )
    parent = value('d405_mount_parent_frame')
    child = value('d405_mount_child_frame')
    minimum_samples = int(value('mount_minimum_sample_count'))
    max_translation = float(value('mount_max_translation_rms_m'))
    max_rotation = float(value('mount_max_rotation_rms_deg'))
    max_outliers = int(value('mount_max_outlier_count'))
    validate_mount_contract(
        start_d405,
        use_mount,
        value('d405_mount_calibration'),
        parent,
        child,
        f"{value('d405_camera_name')}_link",
        minimum_samples,
        max_translation,
        max_rotation,
        max_outliers,
    )

    actions = []
    if use_mount:
        actions.append(Node(
            package='dual_camera_foundation',
            executable='calibrated_mount_tf_publisher',
            name='calibrated_wrist_camera_mount_publisher',
            output='screen',
            parameters=[{
                'calibration_file': value('d405_mount_calibration'),
                'parent_frame': parent,
                'child_frame': child,
                'minimum_sample_count': minimum_samples,
                'max_translation_rms_m': max_translation,
                'max_rotation_rms_deg': max_rotation,
                'max_outlier_count': max_outliers,
            }],
        ))
    if start_astra:
        actions.append(Node(
            package='camera_perception',
            executable='target_transform_node',
            name='astra_target_transform_node',
            output='screen',
            parameters=[{
                'input_topic': value('astra_input_topic'),
                'output_topic': value('astra_output_topic'),
                'target_frame': value('target_frame'),
                'use_latest_transform': True,
                'max_message_age_sec': float(
                    value('astra_max_message_age_sec')
                ),
                'future_tolerance_sec': float(
                    value('astra_future_tolerance_sec')
                ),
                'transform_timeout_sec': float(
                    value('astra_transform_timeout_sec')
                ),
                'publish_marker': _as_bool(
                    value('publish_marker'),
                    'publish_marker',
                ),
                'marker_topic': '/perception/astra_target_marker',
            }],
        ))
    if start_d405:
        actions.append(Node(
            package='camera_perception',
            executable='target_transform_node',
            name='d405_target_transform_node',
            output='screen',
            parameters=[{
                'input_topic': value('d405_input_topic'),
                'output_topic': value('d405_output_topic'),
                'target_frame': value('target_frame'),
                'use_latest_transform': False,
                'max_message_age_sec': float(
                    value('d405_max_message_age_sec')
                ),
                'future_tolerance_sec': float(
                    value('d405_future_tolerance_sec')
                ),
                'transform_timeout_sec': float(
                    value('d405_transform_timeout_sec')
                ),
                'publish_marker': _as_bool(
                    value('publish_marker'),
                    'publish_marker',
                ),
                'marker_topic': '/perception/d405_target_marker',
            }],
        ))
    actions.append(Node(
        package='dual_camera_foundation',
        executable='dual_camera_refiner',
        name='dual_camera_refiner',
        output='screen',
        parameters=[{
            'astra_topic': value('astra_output_topic'),
            'd405_topic': value('d405_output_topic'),
            'output_topic': value('output_topic'),
            'astra_max_age_sec': float(value('astra_refiner_max_age_sec')),
            'd405_max_age_sec': float(value('d405_refiner_max_age_sec')),
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
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_astra_transform',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_d405_transform',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'use_mount_calibration',
            default_value='false',
        ),
        DeclareLaunchArgument('d405_mount_calibration', default_value=''),
        DeclareLaunchArgument(
            'd405_mount_parent_frame',
            default_value='tool0',
        ),
        DeclareLaunchArgument(
            'd405_mount_child_frame',
            default_value='wrist_d405_link',
        ),
        DeclareLaunchArgument(
            'd405_camera_name',
            default_value='wrist_d405',
        ),
        DeclareLaunchArgument(
            'mount_minimum_sample_count',
            default_value='15',
        ),
        DeclareLaunchArgument(
            'mount_max_translation_rms_m',
            default_value='0.01',
        ),
        DeclareLaunchArgument(
            'mount_max_rotation_rms_deg',
            default_value='2.0',
        ),
        DeclareLaunchArgument(
            'mount_max_outlier_count',
            default_value='0',
        ),
        DeclareLaunchArgument(
            'astra_input_topic',
            default_value='/camera/tag_point',
        ),
        DeclareLaunchArgument(
            'astra_output_topic',
            default_value='/perception/astra_target_point_base',
        ),
        DeclareLaunchArgument(
            'd405_input_topic',
            default_value='/perception/d405_target_point_camera',
        ),
        DeclareLaunchArgument(
            'd405_output_topic',
            default_value='/perception/d405_target_point_base',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/perception/dual_target_point_base',
        ),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_marker', default_value='false'),
        DeclareLaunchArgument(
            'astra_max_message_age_sec',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'd405_max_message_age_sec',
            default_value='0.35',
        ),
        DeclareLaunchArgument(
            'astra_future_tolerance_sec',
            default_value='0.1',
        ),
        DeclareLaunchArgument(
            'd405_future_tolerance_sec',
            default_value='0.05',
        ),
        DeclareLaunchArgument(
            'astra_transform_timeout_sec',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'd405_transform_timeout_sec',
            default_value='0.2',
        ),
        DeclareLaunchArgument(
            'astra_refiner_max_age_sec',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'd405_refiner_max_age_sec',
            default_value='0.35',
        ),
        DeclareLaunchArgument(
            'refiner_future_tolerance_sec',
            default_value='0.05',
        ),
        DeclareLaunchArgument(
            'refiner_max_pair_time_delta_sec',
            default_value='0.25',
        ),
        DeclareLaunchArgument(
            'refiner_max_pair_distance_m',
            default_value='0.08',
        ),
        DeclareLaunchArgument(
            'refiner_max_output_jump_m',
            default_value='0.08',
        ),
        DeclareLaunchArgument(
            'refiner_output_jump_reference_max_age_sec',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'refiner_require_astra_reference',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'refiner_require_same_frame',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'allow_astra_fallback',
            default_value='true',
        ),
        OpaqueFunction(function=_setup),
    ])
