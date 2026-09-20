"""Conversions and RViz messages only; no planning or robot commands."""

import numpy as np
import open3d as o3d
from geometry_msgs.msg import Point, PoseStamped, Vector3
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Box, matrix_from_pose, pose_from_box, pose_from_matrix


def vector(values):
    return Vector3(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def cloud_message(header, points):
    return create_cloud_xyz32(header, np.asarray(points, dtype=np.float32).reshape(-1, 3))


def cloud_xyz(message):
    """Handle sensor_msgs_py releases returning structured arrays or tuples."""
    rows = read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)
    if isinstance(rows, np.ndarray) and rows.dtype.names:
        return np.column_stack([rows[name].ravel() for name in ('x', 'y', 'z')]).astype(float)
    return np.asarray(list(rows), dtype=float).reshape(-1, 3)


def box_from_message(message):
    pose = matrix_from_pose(message.pose)
    dims = message.dimensions
    dimensions = np.array([dims.x, dims.y, dims.z])
    if not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
        raise ValueError('object dimensions must be finite and positive')
    return Box(pose[:3, 3], pose[:3, :3], dimensions, message.point_count,
               message.quality, message.yaw_observable)


def scene_message(header, observation):
    from tabletop_grasp_interfaces.msg import OrientedObject, TabletopScene
    msg = TabletopScene(header=header, valid=observation.valid, reason=observation.reason,
                        target_index=observation.target_index)
    if observation.table:
        msg.table_pose = pose_from_box(observation.table)
        msg.table_dimensions = vector(observation.table.dimensions)
        msg.table_point_count = observation.table.point_count
        msg.table_inlier_ratio = observation.plane_ratio
    for index, box in enumerate(observation.boxes):
        msg.objects.append(OrientedObject(
            id=f'object_{index}', pose=pose_from_box(box), dimensions=vector(box.dimensions),
            point_count=int(box.point_count), quality=float(box.quality),
            yaw_observable=bool(box.yaw_observable)))
    return msg


def stamped(header, matrix):
    return PoseStamped(header=header, pose=pose_from_matrix(matrix))


class Markers:
    def __init__(self, header, namespace, lifetime=1.5):
        self.header, self.namespace, self.lifetime = header, namespace, lifetime
        self.array = MarkerArray(markers=[Marker(header=header, action=Marker.DELETEALL)])

    def add(self, kind, color):
        marker = Marker(header=self.header, ns=self.namespace, id=len(self.array.markers),
                        type=kind, action=Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, color)
        marker.lifetime.sec = int(self.lifetime)
        marker.lifetime.nanosec = int((self.lifetime % 1) * 1e9)
        self.array.markers.append(marker)
        return marker

    def box(self, box, color, outline=False):
        marker = self.add(Marker.CUBE, color)
        marker.pose, marker.scale = pose_from_box(box), vector(box.dimensions)
        if outline:
            lines = o3d.geometry.LineSet.create_from_oriented_bounding_box(box.obb())
            xyz = np.asarray(lines.points)[np.asarray(lines.lines).ravel()]
            marker = self.add(Marker.LINE_LIST, (*color[:3], 1.0))
            marker.scale.x = 0.002
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in xyz]

    def arrow(self, start, end, color, width=0.004):
        marker = self.add(Marker.ARROW, color)
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (start, end)]
        marker.scale = vector([width, width * 2, width * 3])

    def axes(self, position, rotation, size=0.06):
        for index, color in enumerate(((1, 0.2, 0.2, 1), (0.2, 1, 0.2, 1), (0.2, 0.4, 1, 1))):
            self.arrow(position, position + size * rotation[:, index], color)

    def label(self, position, text, color=(1, 1, 1, 1), height=0.018):
        marker = self.add(Marker.TEXT_VIEW_FACING, color)
        marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        marker.scale.z = height
        marker.text = text


def observation_markers(header, observation, lifetime, label_height=0.018):
    markers = Markers(header, 'perception', lifetime)
    if observation.table:
        markers.box(observation.table, (0.3, 0.5, 1, 0.25))
    for index, box in enumerate(observation.boxes):
        if box.observed_bounds is not None:
            markers.box(box.observed_bounds, (0.65, 0.65, 0.65, 0.1), outline=True)
        markers.box(box, (1, 0.6, 0.1, 0.3), outline=True)
        markers.axes(box.center, box.rotation)
        center = markers.add(Marker.SPHERE, (1, 1, 1, 1))
        center.pose = pose_from_box(box)
        center.scale = vector([0.008] * 3)
        label = (f'{index}: top support mm={np.round(box.dimensions * 1000, 1)}\n'
                 'FULL EXTENT UNVERIFIED' if box.observed_bounds is not None else
                 f'{index}: long/short/normal mm={np.round(box.dimensions * 1000, 1)}\n'
                 f'n={box.point_count}, support={box.quality:.2f}, yaw={box.yaw_observable}')
        horizontal_offset, clearance = ((0.05, 0.01) if box.observed_bounds is not None else (0.09, 0.03))
        markers.label(box.center + box.rotation[:, 0] * horizontal_offset
                      + box.rotation[:, 2] * (box.dimensions[2] / 2 + clearance),
                      label, height=label_height)
    return markers.array


def orientation_preview_markers(header, observation, lifetime, label_height=0.018):
    """Two equivalent closing directions, with no TCP, jaw or grasp-depth claim.

    Arrows originate at the reconstructed object center, not at a selected
    physical TCP. Their opposite directions encode the pi symmetry of a
    parallel-jaw orientation. No trajectory or feasible grasp pose is output.
    """
    markers = Markers(header, 'unvalidated_orientation_only', lifetime)
    if observation.valid:
        box = observation.boxes[observation.target_index]
        if box.yaw_observable:
            for sign, color in ((1, (0.1, 1.0, 0.9, 1)), (-1, (0.9, 0.2, 1.0, 1))):
                markers.arrow(box.center, box.center + sign * 0.065 * box.rotation[:, 1],
                              color, width=0.003)
            markers.label(box.center + 0.055 * box.rotation[:, 2],
                          '+/- SHORT AXIS\nORIENTATION ONLY',
                          color=(1, 0.9, 0.2, 1), height=label_height)
    return markers.array
