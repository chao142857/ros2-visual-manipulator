"""Camera-frame geometry only: no fabricated robot state, extrinsics or planner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('tabletop_grasp'))
    wrist = Path(get_package_share_directory('wrist_camera_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(package / 'config/camera_observation.yaml')),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_serial', default_value='1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(wrist / 'launch/wrist_d405.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera')),
            launch_arguments={'serial_no': LaunchConfiguration('camera_serial'),
                              'align_depth': 'true', 'pointcloud': 'false',
                              'publish_uncalibrated_mount_tf': 'false'}.items()),
        Node(package='tabletop_grasp', executable='perception', name='tabletop_camera_perception',
             parameters=[LaunchConfiguration('config'),
                         {'frame_mode': 'camera', 'output_namespace': '/tabletop_camera',
                          'object_frame': 'tabletop_camera_object'}],
             output='screen', additional_env={'OMP_NUM_THREADS': '2'}),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(package / 'config/camera_observation.rviz')]),
    ])
