            
"""Launch the dedicated D405 AprilTag pose publisher without a GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic_prefix', default_value='/wrist_camera/wrist_d405'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('rgb_topic', default_value=''),
        DeclareLaunchArgument('camera_info_topic', default_value=''),
        DeclareLaunchArgument('pose_topic', default_value=''),
        DeclareLaunchArgument('target_pixel_topic', default_value=''),
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument('pixel_frame', default_value=''),
        DeclareLaunchArgument('target_frame', default_value='wrist_calibration_target_6'),
        DeclareLaunchArgument('target_tag_id', default_value='6'),
        DeclareLaunchArgument('tag_size_m', default_value='0.0'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_target_pixel', default_value='true'),
        DeclareLaunchArgument('queue_depth', default_value='5'),
        DeclareLaunchArgument('min_distance_m', default_value='0.03'),
        DeclareLaunchArgument('max_distance_m', default_value='3.0'),
        Node(
            package='dual_camera_foundation', executable='d405_apriltag_pose',
            name='d405_apriltag_pose', output='screen',
            parameters=[{
                'topic_prefix': LaunchConfiguration('topic_prefix'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'target_pixel_topic': LaunchConfiguration('target_pixel_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'pixel_frame': LaunchConfiguration('pixel_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
                'target_tag_id': ParameterValue(LaunchConfiguration('target_tag_id'), value_type=int),
                'tag_size_m': ParameterValue(LaunchConfiguration('tag_size_m'), value_type=float),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                'publish_target_pixel': ParameterValue(LaunchConfiguration('publish_target_pixel'), value_type=bool),
                'queue_depth': ParameterValue(LaunchConfiguration('queue_depth'), value_type=int),
                'min_distance_m': ParameterValue(LaunchConfiguration('min_distance_m'), value_type=float),
                'max_distance_m': ParameterValue(LaunchConfiguration('max_distance_m'), value_type=float),
            }],
        ),
    ])

        

