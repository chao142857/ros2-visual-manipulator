"""Open the inspection-only RViz view with the frozen robot model supplied."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('tabletop_grasp'))
    robot = Path(get_package_share_directory('arm_description')) / 'urdf/dummy-v2.urdf'
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2',
             parameters=[{'robot_description': robot.read_text(),
                          'robot_description_semantic': (package / 'config/dummy_v2.srdf').read_text()}],
             arguments=['-d', str(package / 'config/tabletop.rviz')]),
    ])
