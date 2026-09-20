"""Detached, unverified gripper articulation for offline visual inspection."""

import json
import math
from pathlib import Path, PurePosixPath

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from tabletop_grasp.gripper_motion import load_parts, MotionModel


FRAME_ID = 'gripper_cad_estimate'
GROUP_NAMES = (
    'fixed', 'left_drive', 'right_drive', 'left_inner', 'right_inner',
    'left_finger', 'right_finger',
)


def validate_parts(parts):
    """Require native-CAD bounds in mm and metre-scale package mesh resources."""
    groups = parts['groups']
    if (len(groups) != len(GROUP_NAMES)
            or {group['name'] for group in groups} != set(GROUP_NAMES)):
        raise ValueError('parts must contain each of the seven gripper groups once')
    for group in groups:
        mesh = PurePosixPath(group['mesh'])
        if (mesh.is_absolute() or '..' in mesh.parts or len(mesh.parts) != 3
                or mesh.parts[:2] != ('meshes', 'gripper_motion') or mesh.suffix != '.stl'):
            raise ValueError('preview meshes must be package gripper_motion STL resources')
        bounds = group['bounds_mm']
        if (len(bounds) != 6 or not all(math.isfinite(value) for value in bounds)
                or any(bounds[index + 3] <= bounds[index] for index in range(3))):
            raise ValueError('bounds_mm must contain finite min XYZ then max XYZ')
    return groups


def _position(pose, coordinates):
    pose.position.x, pose.position.y, pose.position.z = (
        float(value) / 1000.0 for value in coordinates)


def _rotation(pose, angle):
    pose.orientation.z = math.sin(angle / 2.0)
    pose.orientation.w = math.cos(angle / 2.0)


def motion_markers(parts, state, stamp, show_bounds=True):
    """Transform original CAD meshes and candidate boxes without publishing TF."""
    groups = validate_parts(parts)
    if state['status'] != 'approximate_unverified':
        raise ValueError('this preview only displays an approximate_unverified model')
    result = MarkerArray()

    def add(namespace, index, kind, color):
        marker = Marker(ns=namespace, id=index, type=kind, action=Marker.ADD)
        marker.header.frame_id, marker.header.stamp = FRAME_ID, stamp
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        result.markers.append(marker)
        return marker

    for index, group in enumerate(groups):
        transform = state['transforms'][group['name']]
        offset = transform['translation_mm']
        angle = transform['rotation_z_rad']
        if (len(offset) != 3 or not all(math.isfinite(value) for value in offset)
                or not math.isfinite(angle)):
            raise ValueError('invalid group rigid transform')
        color = ((0.58, 0.63, 0.68, 1.0) if group['name'] == 'fixed'
                 else (0.18, 0.66, 0.95, 1.0) if group['name'].startswith('left')
                 else (0.45, 0.85, 0.48, 1.0))
        mesh = add('estimate_mesh', index, Marker.MESH_RESOURCE, color)
        mesh.mesh_resource = f'package://tabletop_grasp/{group["mesh"]}'
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
        _position(mesh.pose, offset)
        _rotation(mesh.pose, angle)

        box = add('candidate_bounds', index, Marker.CUBE, (1.0, 0.52, 0.12, 0.16))
        if not show_bounds:
            # Explicit deletion also removes boxes from a previously shown pose.
            box.action = Marker.DELETE
            continue
        bounds = group['bounds_mm']
        center = [(bounds[axis] + bounds[axis + 3]) / 2.0 for axis in range(3)]
        cosine, sine = math.cos(angle), math.sin(angle)
        moved = [cosine * center[0] - sine * center[1] + offset[0],
                 sine * center[0] + cosine * center[1] + offset[1],
                 center[2] + offset[2]]
        _position(box.pose, moved)
        _rotation(box.pose, angle)
        box.scale.x, box.scale.y, box.scale.z = (
            float(bounds[axis + 3] - bounds[axis]) / 1000.0 for axis in range(3))

    label = add('estimate_label', 0, Marker.TEXT_VIEW_FACING, (1.0, 0.78, 0.20, 1.0))
    label.pose.position.y, label.pose.position.z = -0.03, 0.10
    label.scale.z = 0.0055
    label.text = (
        'UNVERIFIED - approximate gripper motion\n'
        f'{state["pulse_us"]:g} us | opening {state["opening_mm"]:.1f} mm | '
        f'TCP extension {state["tcp_extension_mm"]:.1f} mm\n'
        'Detached CAD frame; physical limits / attachment unverified\n'
        'Orange: candidate group bounds; collision checks not performed')
    return result


def validate_preview_values(model, pulse_us, show_bounds):
    """Reject malformed user input before changing the displayed estimate."""
    if isinstance(pulse_us, bool) or not isinstance(pulse_us, (int, float)):
        raise ValueError('pulse_us must be a finite number in the configured range')
    if not math.isfinite(pulse_us):
        raise ValueError('pulse_us must be finite')
    if not isinstance(show_bounds, bool):
        raise ValueError('show_bounds must be true or false')
    return model.state(pulse_us)


class GripperMotionPreviewNode(Node):
    """Publish only display/state outputs plus standard ROS parameter services."""

    def __init__(self):
        super().__init__('gripper_motion_preview', enable_rosout=False)
        package = Path(get_package_share_directory('tabletop_grasp'))
        self.declare_parameter(
            'config_path', str(package / 'config/gripper_motion_estimate.json'),
            ParameterDescriptor(read_only=True, description='Unverified motion estimate JSON'))
        self.declare_parameter(
            'parts_path', str(package / 'config/gripper_motion_parts.json'),
            ParameterDescriptor(read_only=True, description='Native CAD mesh group manifest'))
        self.declare_parameter(
            'pulse_us', 1300.0,
            ParameterDescriptor(dynamic_typing=True,
                                description='Offline input only; model range is validated'))
        self.declare_parameter('show_bounds', True)
        self.model = MotionModel.from_json(self.get_parameter('config_path').value)
        self.parts = load_parts(self.get_parameter('parts_path').value, self.model)
        validate_parts(self.parts)
        validate_preview_values(self.model, self.get_parameter('pulse_us').value,
                                self.get_parameter('show_bounds').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/gripper_estimate/markers', qos)
        self.state_pub = self.create_publisher(String, '/gripper_estimate/state', qos)
        self._refresh = self.create_guard_condition(self._publish_current)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self._publish_current()
        self.get_logger().info(
            'UNVERIFIED gripper preview; edit pulse_us to inspect a fixed estimated pose.')

    def _validate_parameters(self, parameters):
        values = {name: self.get_parameter(name).value
                  for name in ('pulse_us', 'show_bounds')}
        changed = False
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
                changed = True
        try:
            validate_preview_values(self.model, values['pulse_us'], values['show_bounds'])
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        if changed:
            # The mutually exclusive executor runs this after parameter commit.
            # Reading committed values also protects against later callback rejection.
            self._refresh.trigger()
        return SetParametersResult(successful=True)

    def _publish_current(self):
        state = validate_preview_values(self.model, self.get_parameter('pulse_us').value,
                                        self.get_parameter('show_bounds').value)
        markers = motion_markers(self.parts, state, self.get_clock().now().to_msg(),
                                 self.get_parameter('show_bounds').value)
        self.marker_pub.publish(markers)
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GripperMotionPreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
