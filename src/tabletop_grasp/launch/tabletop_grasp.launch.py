"""Normal tag-free RGB-D entry point. Hardware execution cannot be enabled."""

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
    robot = Path(get_package_share_directory('arm_description')) / 'urdf/dummy-v2.urdf'
    config = LaunchConfiguration('config')
    joints = LaunchConfiguration('joint_states_topic')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(package / 'config/tabletop.yaml')),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_serial', default_value='1'),
        DeclareLaunchArgument('start_robot_state', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('start_planner', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        # User confirmed unchanged wrist mount; existing calibration is provisional.
        DeclareLaunchArgument('publish_extrinsics', default_value='true'),
        DeclareLaunchArgument('calibration_file',
                              default_value=str(wrist / 'config/d405_mount_calibration.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(wrist / 'launch/wrist_d405.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera')),
            launch_arguments={'serial_no': LaunchConfiguration('camera_serial'),
                              'align_depth': 'true', 'pointcloud': 'false',
                              'publish_uncalibrated_mount_tf': 'false'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             condition=IfCondition(LaunchConfiguration('start_robot_state')),
             parameters=[{'robot_description': robot.read_text()}],
             remappings=[('joint_states', joints)]),
        Node(package='tabletop_grasp', executable='extrinsics',
             condition=IfCondition(LaunchConfiguration('publish_extrinsics')),
             parameters=[{'calibration_file': LaunchConfiguration('calibration_file')}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/moveit_planning.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_moveit')),
            launch_arguments={'joint_states_topic': joints}.items()),
        Node(package='tabletop_grasp', executable='perception', name='tabletop_perception',
             parameters=[config], output='screen', additional_env={'OMP_NUM_THREADS': '2'}),
        Node(package='tabletop_grasp', executable='planner', name='tabletop_planner',
             condition=IfCondition(LaunchConfiguration('start_planner')),
             parameters=[config, {'dry_run': True, 'joint_states_topic': joints}], output='screen',
             additional_env={'OMP_NUM_THREADS': '2'}),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'robot_description': robot.read_text(),
                          'robot_description_semantic': (package / 'config/dummy_v2.srdf').read_text()}],
             arguments=['-d', str(package / 'config/tabletop.rviz')]),
    ])
