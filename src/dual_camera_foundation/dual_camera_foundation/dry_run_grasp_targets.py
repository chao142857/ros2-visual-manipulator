            
"""Publish conservative preview-only pregrasp, grasp, and retract poses."""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node


class DryRunGraspTargets(Node):
    """Generate Cartesian targets and never command hardware."""

    def __init__(self):
        super().__init__('dry_run_grasp_targets')
        self.declare_parameter('input_topic', '/perception/dual_target_point_base')
        self.declare_parameter('pregrasp_offset_z', 0.10)
        self.declare_parameter('retract_offset_z', 0.12)
        self.declare_parameter('output_prefix', '/perception/dry_run')
        prefix = str(self.get_parameter('output_prefix').value).rstrip('/')
        self.pregrasp = self.create_publisher(PoseStamped, prefix + '/pregrasp', 10)
        self.grasp = self.create_publisher(PoseStamped, prefix + '/grasp', 10)
        self.retract = self.create_publisher(PoseStamped, prefix + '/retract', 10)
        self.create_subscription(PointStamped, self.get_parameter('input_topic').value,
                                 self.callback, 10)

    @staticmethod
    def pose(point, offset):
        message = PoseStamped()
        message.header = point.header
        message.pose.position.x = point.point.x
        message.pose.position.y = point.point.y
        message.pose.position.z = point.point.z + offset


        # Fixed top-grasp orientation, obtained from PREVIEW RViz.
        message.pose.orientation.x = 0.724
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = 0.0
        message.pose.orientation.w = 0.690

        
        return message

    def callback(self, point):
        if not all(math.isfinite(value) for value in (point.point.x, point.point.y, point.point.z)):
            return
        self.pregrasp.publish(self.pose(point, float(self.get_parameter('pregrasp_offset_z').value)))
        self.grasp.publish(self.pose(point, 0.0))
        self.retract.publish(self.pose(point, float(self.get_parameter('retract_offset_z').value)))


def main(args=None):
    rclpy.init(args=args)
    node = DryRunGraspTargets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

