"""Detached native-CAD inspection markers; no robot, TF or planning interface."""

import itertools
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker, MarkerArray


def cad_markers(proxy, stamp):
    """Display metre-scale geometry in gripper_cad; this is not an attachment."""
    if (proxy['units'] != 'm' or proxy['frame_id'] != 'gripper_cad'
            or proxy['conservative'] is not True):
        raise ValueError('CAD preview requires conservative metre-scale gripper_cad geometry')
    result = MarkerArray()

    def add(namespace, kind, color):
        marker = Marker(id=len(result.markers), ns=namespace, type=kind, action=Marker.ADD)
        marker.header.frame_id, marker.header.stamp = 'gripper_cad', stamp
        marker.pose.orientation.w = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        result.markers.append(marker)
        return marker

    mesh = add('cad_visual', Marker.MESH_RESOURCE, (0.68, 0.75, 0.84, 0.85))
    mesh.mesh_resource = 'package://tabletop_grasp/meshes/cad_gripper_visual.stl'
    mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0  # STL already exported in metres.
    for box in proxy['boxes']:
        dimensions, center = np.asarray(box['dimensions']), np.asarray(box['center'])
        rotation = Rotation.from_quat(box['quaternion_xyzw'])
        if (dimensions.shape != (3,) or center.shape != (3,)
                or not np.isfinite(dimensions).all() or not np.isfinite(center).all()
                or np.any(dimensions <= 0)):
            raise ValueError('invalid CAD preview box')
        solid = add('cad_collision', Marker.CUBE, (1., 0.52, 0.10, 0.14))
        solid.pose.position.x, solid.pose.position.y, solid.pose.position.z = map(float, center)
        quaternion = solid.pose.orientation
        quaternion.x, quaternion.y, quaternion.z, quaternion.w = map(float, rotation.as_quat())
        solid.scale.x, solid.scale.y, solid.scale.z = map(float, dimensions)
        lines = add('cad_collision_edges', Marker.LINE_LIST, (1., 0.55, 0.12, 0.95))
        lines.scale.x = 0.00045
        signs = np.array(list(itertools.product((-1, 1), repeat=3)))
        corners = rotation.apply(signs * dimensions / 2) + center
        for first in range(8):
            for second in range(first + 1, 8):
                if np.count_nonzero(signs[first] != signs[second]) == 1:
                    for point in (corners[first], corners[second]):
                        lines.points.append(Point(x=float(point[0]), y=float(point[1]),
                                                  z=float(point[2])))
    for axis, color in enumerate(((1., 0.2, 0.2, 1.), (0.2, 1., 0.2, 1.), (0.2, 0.5, 1., 1.))):
        arrow = add('cad_native_axes', Marker.ARROW, color)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.001, 0.003, 0.005
        end = np.eye(3)[axis] * 0.03
        arrow.points = [Point(), Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
    label = add('cad_label', Marker.TEXT_VIEW_FACING, (1., 1., 1., 1.))
    label.pose.position.y, label.pose.position.z = -0.045, 0.085
    label.scale.z = 0.006
    margin_mm = 1000 * proxy['safety_margin_m']
    label.text = ('STATIC CAD - detached native frame (metres)\n'
                  'TCP / attachment / jaw motion UNVERIFIED\n'
                  f'Orange: {len(proxy["boxes"])} collision boxes, +{margin_mm:g} mm per side')
    return result


class CadPreviewNode(Node):
    def __init__(self):
        super().__init__('tabletop_cad_preview', enable_rosout=False, start_parameter_services=False)
        path = Path(get_package_share_directory('tabletop_grasp')) / 'config/cad_collision_proxy.json'
        self.proxy = json.loads(path.read_text())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/tabletop_cad/markers', qos)
        self.create_timer(1.0, self._publish)
        self._publish()

    def _publish(self):
        self.markers.publish(cad_markers(self.proxy, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = CadPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
