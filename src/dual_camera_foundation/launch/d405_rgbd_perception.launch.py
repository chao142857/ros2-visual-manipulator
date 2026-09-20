"""Connect D405 AprilTag pixels to aligned RGB-D points and TF.

This launch owns perception nodes only. The wrist RealSense driver remains
owned by ``wrist_camera_bringup`` and must be started with
``align_depth:=true`` when the default aligned-depth topics are used.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the D405 perception chain without starting camera hardware."""
    start_apriltag = LaunchConfiguration('start_apriltag')
    start_rgbd = LaunchConfiguration('start_rgbd')
    start_transform = LaunchConfiguration('start_transform')
    return LaunchDescription([
        DeclareLaunchArgument('start_apriltag', default_value='true'),
        DeclareLaunchArgument('start_rgbd', default_value='true'),
        DeclareLaunchArgument('start_transform', default_value='true'),
        DeclareLaunchArgument(
            'topic_prefix',
            default_value='/wrist_camera/wrist_d405',
        ),
        DeclareLaunchArgument('rgb_topic', default_value=''),
        DeclareLaunchArgument('color_info_topic', default_value=''),
        DeclareLaunchArgument('pixel_frame', default_value=''),
        DeclareLaunchArgument(
            'calibration_target_frame',
            default_value='wrist_calibration_target_6',
        ),
        DeclareLaunchArgument('target_pose_topic', default_value=''),
        DeclareLaunchArgument(
            'base_pose_topic',
            default_value='/perception/d405_target_pose_base',
        ),
        DeclareLaunchArgument(
            'target_pixel_topic',
            default_value=(
                '/wrist_camera/wrist_d405/'
                'calibration/target_pixel'
            ),
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=(
                '/wrist_camera/wrist_d405/'
                'aligned_depth_to_color/image_raw'
            ),
        ),
        DeclareLaunchArgument(
            'depth_info_topic',
            default_value=(
                '/wrist_camera/wrist_d405/'
                'aligned_depth_to_color/camera_info'
            ),
        ),
        DeclareLaunchArgument(
            'point_topic',
            default_value='/perception/d405_target_point_camera',
        ),
        DeclareLaunchArgument(
            'base_point_topic',
            default_value='/perception/d405_target_point_base',
        ),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_marker', default_value='false'),
        DeclareLaunchArgument('target_tag_id', default_value='6'),
        DeclareLaunchArgument('tag_size_m', default_value='0.0'),
        DeclareLaunchArgument(
            'max_depth_age_sec',
            default_value='0.05',
            description='Maximum age of an eligible older depth frame.',
        ),
        DeclareLaunchArgument(
            'future_tolerance_sec',
            default_value='0.05',
            description='Maximum lead of an eligible newer depth frame.',
        ),
        DeclareLaunchArgument(
            'depth_queue_size',
            default_value='8',
            description='Bounded depth history used for nearest-frame match.',
        ),
        Node(
            package='dual_camera_foundation',
            executable='d405_apriltag_pose',
            name='d405_apriltag_pose',
            output='screen',
            condition=IfCondition(start_apriltag),
            parameters=[{
                'topic_prefix': LaunchConfiguration('topic_prefix'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'camera_info_topic': LaunchConfiguration(
                    'color_info_topic'
                ),
                'pose_topic': LaunchConfiguration('target_pose_topic'),
                'target_pixel_topic': LaunchConfiguration(
                    'target_pixel_topic'
                ),
                'pixel_frame': LaunchConfiguration('pixel_frame'),
                'target_frame': LaunchConfiguration(
                    'calibration_target_frame'
                ),
                'target_tag_id': ParameterValue(
                    LaunchConfiguration('target_tag_id'),
                    value_type=int,
                ),
                'tag_size_m': ParameterValue(
                    LaunchConfiguration('tag_size_m'),
                    value_type=float,
                ),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'),
                    value_type=bool,
                ),
                'publish_target_pixel': True,
            }],
        ),
        Node(
            package='dual_camera_foundation',
            executable='rgbd_target_point',
            name='d405_rgbd_target_point',
            output='screen',
            condition=IfCondition(start_rgbd),
            parameters=[{
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration(
                    'depth_info_topic'
                ),
                'pixel_topic': LaunchConfiguration('target_pixel_topic'),
                'output_topic': LaunchConfiguration('point_topic'),
                'max_depth_age_sec': ParameterValue(
                    LaunchConfiguration('max_depth_age_sec'),
                    value_type=float,
                ),
                'future_tolerance_sec': ParameterValue(
                    LaunchConfiguration('future_tolerance_sec'),
                    value_type=float,
                ),
                'depth_queue_size': ParameterValue(
                    LaunchConfiguration('depth_queue_size'),
                    value_type=int,
                ),
            }],
        ),
        Node(
            package='dual_camera_foundation',
            executable='target_pose_transform',
            name='d405_target_pose_transform_node',
            output='screen',
            condition=IfCondition(start_transform),
            parameters=[{
                'input_topic': LaunchConfiguration('target_pose_topic'),
                'output_topic': LaunchConfiguration('base_pose_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
            }],
        ),
        Node(
            package='camera_perception',
            executable='target_transform_node',
            name='d405_target_transform_node',
            output='screen',
            condition=IfCondition(start_transform),
            parameters=[{
                'input_topic': LaunchConfiguration('point_topic'),
                'output_topic': LaunchConfiguration('base_point_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                # A moving wrist must query TF at the depth observation stamp.
                'use_latest_transform': False,
                'publish_marker': ParameterValue(
                    LaunchConfiguration('publish_marker'),
                    value_type=bool,
                ),
                'marker_topic': '/perception/d405_target_marker',
            }],
        ),
    ])
