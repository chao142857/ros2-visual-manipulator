"""Private MoveIt planning server: no drivers, controllers or execution capability."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    package = Path(get_package_share_directory('tabletop_grasp'))
    robot = Path(get_package_share_directory('arm_description')) / 'urdf/dummy-v2.urdf'
    parameters = {
        'robot_description': robot.read_text(),
        'robot_description_semantic': (package / 'config/dummy_v2.srdf').read_text(),
        'robot_description_kinematics': yaml.safe_load((package / 'config/kinematics.yaml').read_text()),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': yaml.safe_load((package / 'config/ompl.yaml').read_text()),
        'allow_trajectory_execution': False,
        'moveit_manage_controllers': False,
        'disable_capabilities': ('move_group/MoveGroupExecuteTrajectoryAction '
                                 'move_group/MoveGroupMoveAction'),
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        'monitor_dynamics': False,
    }
    return LaunchDescription([
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        Node(package='moveit_ros_move_group', executable='move_group', namespace='tabletop',
             name='move_group', output='screen', parameters=[parameters],
             remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))]),
    ])
