"""Software-only MoveIt/point-cloud fixture; use ROS_DOMAIN_ID=191."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '191':
        raise RuntimeError('Synthetic demo requires ROS_DOMAIN_ID=191 to isolate test TF and states')
    package = Path(get_package_share_directory('tabletop_grasp'))
    robot = Path(get_package_share_directory('arm_description')) / 'urdf/dummy-v2.urdf'
    parameters = [str(package / 'config/tabletop.yaml'), str(package / 'config/synthetic.yaml')]
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='tabletop_grasp', executable='synthetic_scene', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot.read_text()}],
             remappings=[('joint_states', '/tabletop/test_joint_states')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/moveit_planning.launch.py')),
            launch_arguments={'joint_states_topic': '/tabletop/test_joint_states'}.items()),
        Node(package='tabletop_grasp', executable='perception', parameters=parameters,
             output='screen', additional_env={'OMP_NUM_THREADS': '2'}),
        Node(package='tabletop_grasp', executable='planner', parameters=parameters,
             output='screen', additional_env={'OMP_NUM_THREADS': '2'}),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'robot_description': robot.read_text(),
                          'robot_description_semantic': (package / 'config/dummy_v2.srdf').read_text()}],
             arguments=['-d', str(package / 'config/tabletop.rviz')]),
    ])
