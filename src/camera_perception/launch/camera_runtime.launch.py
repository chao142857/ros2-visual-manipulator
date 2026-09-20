"""Launch saved fixed-camera extrinsics and base-frame target localization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build normal runtime mode without running calibration."""
    default_extrinsics_file = PathJoinSubstitution([
        FindPackageShare('camera_perception'),
        'config',
        'camera_extrinsics.yaml',
    ])

    extrinsics_file = LaunchConfiguration('extrinsics_file')
    start_transform = LaunchConfiguration('start_transform')
    start_tag_localizer = LaunchConfiguration('start_tag_localizer')
    target_tag_id = LaunchConfiguration('target_tag_id')
    target_input_topic = LaunchConfiguration('target_input_topic')
    target_output_topic = LaunchConfiguration('target_output_topic')
    target_frame = LaunchConfiguration('target_frame')
    max_message_age_sec = LaunchConfiguration('max_message_age_sec')
    publish_marker = LaunchConfiguration('publish_marker')

    extrinsics_publisher = Node(
        package='camera_perception',
        executable='camera_extrinsics_publisher_node',
        name='camera_extrinsics_publisher_node',
        output='screen',
        condition=IfCondition(start_transform),
        parameters=[extrinsics_file],
    )

    tag_localizer = Node(
        package='camera_perception',
        executable='tag_localizer_node',
        name='tag_localizer_node',
        output='screen',
        condition=IfCondition(start_tag_localizer),
        parameters=[{
            'target_tag_id': ParameterValue(target_tag_id, value_type=int),
        }],
    )

    target_transformer = Node(
        package='camera_perception',
        executable='target_transform_node',
        name='target_transform_node',
        output='screen',
        condition=IfCondition(start_transform),
        parameters=[{
            'input_topic': target_input_topic,
            'output_topic': target_output_topic,
            'target_frame': target_frame,

            'use_latest_transform': True,

            'max_message_age_sec': ParameterValue(
                max_message_age_sec,
                value_type=float,
            ),
            'publish_marker': ParameterValue(
                publish_marker,
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=default_extrinsics_file,
            description='Saved camera extrinsics ROS parameter YAML.',
        ),
        DeclareLaunchArgument(
            'start_transform',
            default_value='true',
            description=(
                'Start saved Astra extrinsics and the base-frame point '
                'transformer together.'
            ),
        ),
        DeclareLaunchArgument(
            'start_tag_localizer',
            default_value='true',
            description='Start the existing RGB-D AprilTag localizer.',
        ),
        DeclareLaunchArgument(
            'target_tag_id',
            default_value='6',
            description='AprilTag ID published by the existing point API.',
        ),
        DeclareLaunchArgument(
            'target_input_topic',
            default_value='/camera/tag_point',
            description=(
                'PointStamped input. Runtime bridges the legacy AprilTag '
                'point topic by configuration, without changing that API.'
            ),
        ),
        DeclareLaunchArgument(
            'target_output_topic',
            default_value='/perception/target_point_base',
            description='Transformed PointStamped output topic.',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_link',
            description='TF frame for transformed target points.',
        ),
        DeclareLaunchArgument(
            'max_message_age_sec',
            default_value='1.0',
            description='Drop older target observations; zero disables.',
        ),
        DeclareLaunchArgument(
            'publish_marker',
            default_value='true',
            description='Publish /perception/target_marker for RViz.',
        ),
        extrinsics_publisher,
        tag_localizer,
        target_transformer,
    ])
