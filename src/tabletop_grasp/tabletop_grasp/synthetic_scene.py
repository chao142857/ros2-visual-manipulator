"""Explicit synthetic perception/planning fixture on dedicated test topics.

Never publishes /joint_states, arm commands, gripper commands, or execution
actions. Run in a separate ROS_DOMAIN_ID as documented in the test launch.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Header

from .ros_geometry import cloud_message


def cuboid_scene(seed=7, center=(0.03, -0.28), size=(0.08, 0.034, 0.04), yaw=0.3,
                 noise=0.0003, table_z=-0.012):
    rng = np.random.default_rng(seed)
    tx, ty = np.meshgrid(np.linspace(-0.19, 0.19, 90), np.linspace(-0.47, -0.06, 90))
    table = np.column_stack((tx.ravel(), ty.ravel(), np.full(tx.size, table_z)))
    length, width, height = size
    x, y = np.meshgrid(
        np.linspace(-length / 2, length / 2, 35), np.linspace(-width / 2, width / 2, 23))
    top = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, height)))
    xx, zz = np.meshgrid(np.linspace(-length / 2, length / 2, 35), np.linspace(0.006, height, 14))
    yy, zy = np.meshgrid(np.linspace(-width / 2, width / 2, 23), np.linspace(0.006, height, 14))
    sides = [np.column_stack((xx.ravel(), np.full(xx.size, sign * width / 2), zz.ravel()))
             for sign in (-1, 1)]
    sides += [np.column_stack((np.full(yy.size, sign * length / 2), yy.ravel(), zy.ravel()))
              for sign in (-1, 1)]
    obj = np.concatenate([top] + sides)
    obj = Rotation.from_euler('z', yaw).apply(obj) + [center[0], center[1], table_z]
    # Remove physically occluded table points beneath the object footprint.
    local_table = Rotation.from_euler('z', -yaw).apply(table - [center[0], center[1], table_z])
    table = table[(np.abs(local_table[:, 0]) > length / 2) | (np.abs(local_table[:, 1]) > width / 2)]
    result = np.concatenate([table, obj])
    return result + rng.normal(0, noise, result.shape)


class SyntheticScene(Node):
    def __init__(self):
        super().__init__('tabletop_synthetic_scene')
        self.declare_parameter('joint_positions', [0.0, 1.309, -1.571, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('object_xy', [0.03, -0.24])
        self.declare_parameter('object_yaw', 0.3)
        self.cloud = self.create_publisher(PointCloud2, '/tabletop/test_cloud', qos_profile_sensor_data)
        self.joints = self.create_publisher(JointState, '/tabletop/test_joint_states', qos_profile_sensor_data)
        self.points = cuboid_scene(
            center=self.get_parameter('object_xy').value, yaw=self.get_parameter('object_yaw').value)
        self.create_timer(0.1, self.publish_joints)
        self.create_timer(0.35, self.publish_cloud)
        self.get_logger().info('SYNTHETIC DATA ONLY on /tabletop/test_*; no hardware connection')

    def publish_joints(self):
        self.joints.publish(JointState(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link'),
            name=[f'joint{i}' for i in range(1, 7)] + ['tcp_extension_joint'],
            position=self.get_parameter('joint_positions').value))

    def publish_cloud(self):
        self.cloud.publish(cloud_message(
            Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link'), self.points))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
