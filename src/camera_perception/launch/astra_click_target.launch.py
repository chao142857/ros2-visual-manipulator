"""Launch click localization for an already-running registered Astra."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose click-to-3D and TF without owning the camera driver."""
    default_extrinsics = PathJoinSubstitution([
        FindPackageShare('camera_perception'),
        'config',
        'camera_extrinsics.yaml',
    ])

    extrinsics_file = LaunchConfiguration('extrinsics_file')
    enable_tracking = LaunchConfiguration('enable_optical_flow_tracking')

    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=default_extrinsics,
            description='Saved fixed-Astra extrinsics parameter YAML.',
        ),
        DeclareLaunchArgument(
            'enable_optical_flow_tracking',
            default_value='false',
            description=(
                'Track the clicked pixel between frames. Keep false for '
                'grasping so occlusion cannot move the selected target.'
            ),
        ),
        Node(
            package='camera_perception',
            executable='camera_extrinsics_publisher_node',
            name='camera_extrinsics_publisher_node',
            output='screen',
            parameters=[extrinsics_file],
        ),
        Node(
            package='camera_perception',
            executable='click_target_node',
            name='click_target_node',
            output='screen',
            parameters=[{
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/depth/camera_info',
                'output_topic': '/camera/object_point',
                'expected_registered_frame': 'camera_color_optical_frame',
                'enable_optical_flow_tracking': ParameterValue(
                    enable_tracking,
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package='camera_perception',
            executable='target_transform_node',
            name='astra_click_target_transform_node',
            output='screen',
            parameters=[{
                'input_topic': '/camera/object_point',
                'output_topic': '/perception/target_point_base',
                'target_frame': 'base_link',
                'use_latest_transform': True,
                'max_message_age_sec': 1.0,
                'publish_marker': True,
            }],
        ),
    ])
