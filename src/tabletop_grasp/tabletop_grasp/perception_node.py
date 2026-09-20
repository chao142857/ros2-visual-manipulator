"""Bounded latest-frame ROS adapter for the tabletop point-cloud pipeline."""

from copy import deepcopy
import json
import time

from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tabletop_grasp_interfaces.msg import TabletopScene
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from .config import PerceptionConfig, declare_config
from .geometry import pose_from_box, transform_points
from .pipeline import Observation, TabletopPipeline
from .rgbd import rgbd_to_xyz
from .ros_geometry import (cloud_message, cloud_xyz, observation_markers,
                           orientation_preview_markers, scene_message)
from .segmentation import load_segmenter


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('tabletop_perception')
        self.cfg = declare_config(self, PerceptionConfig, 'perception')
        defaults = {
            'input_mode': 'rgbd', 'base_frame': 'base_link',
            'frame_mode': 'base', 'output_namespace': '',
            'camera_max_incidence_deg': 75.0,
            'color_topic': '/wrist_camera/wrist_d405/color/image_rect_raw',
            'depth_topic': '/wrist_camera/wrist_d405/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/wrist_camera/wrist_d405/color/camera_info',
            'cloud_topic': '/wrist_camera/wrist_d405/depth/color/points',
            'cloud_is_optical': True, 'rgbd_rectified': True,
            'rgbd_inverse_brown_conrady': False,
            'sync_slop_sec': 0.05, 'sync_queue_size': 6,
            'max_age_sec': 1.0, 'future_tolerance_sec': 0.05,
            'processing_period_sec': 0.25, 'marker_lifetime_sec': 1.5,
            'label_height_m': 0.018,
            'segmenter_plugin': '', 'object_frame': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if self.p['input_mode'] not in ('rgbd', 'cloud'):
            raise ValueError('input_mode must be rgbd or cloud')
        camera_mode = self.p['frame_mode'] == 'camera'
        namespace = self.p['output_namespace'] or ('/tabletop_camera' if camera_mode else '/tabletop')
        self.namespace = '/' + namespace.strip('/')
        if camera_mode and self.namespace == '/tabletop':
            raise ValueError('camera observation must use a separate namespace from the base planner')
        self.p['object_frame'] = self.p['object_frame'] or (
            'tabletop_camera_object' if camera_mode else 'tabletop_object')
        for name in ('max_age_sec', 'processing_period_sec', 'marker_lifetime_sec', 'sync_slop_sec', 'label_height_m'):
            if not np.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'invalid {name}')
        if self.p['future_tolerance_sec'] < 0 or self.p['sync_queue_size'] < 2:
            raise ValueError('invalid synchronization/freshness settings')
        self.pipeline = TabletopPipeline(
            self.cfg, load_segmenter(self.p['segmenter_plugin'], self.cfg),
            frame_mode=self.p['frame_mode'],
            camera_max_incidence_deg=self.p['camera_max_incidence_deg'])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.pending = None
        self.last_received = time.monotonic()
        self.last_reason = None
        self.last_source_header = None
        self.scene_pub = self.create_publisher(TabletopScene, f'{self.namespace}/scene', 1)
        self.marker_pub = self.create_publisher(MarkerArray, f'{self.namespace}/perception_markers', 1)
        self.diag_pub = self.create_publisher(DiagnosticArray, f'{self.namespace}/diagnostics', 5)
        self.preview_pub = (self.create_publisher(
            MarkerArray, f'{self.namespace}/orientation_preview', 1) if camera_mode else None)
        self.cloud_pubs = {
            name: self.create_publisher(PointCloud2, f'{self.namespace}/{name}_cloud', qos_profile_sensor_data)
            for name in ('input', 'filtered', 'table', 'target')}
        if self.p['input_mode'] == 'cloud':
            self.subscription = self.create_subscription(
                PointCloud2, self.p['cloud_topic'], lambda msg: self._receive(('cloud', msg)),
                qos_profile_sensor_data)
        else:
            self.subs = [message_filters.Subscriber(self, kind, self.p[name],
                                                    qos_profile=qos_profile_sensor_data)
                         for kind, name in ((Image, 'color_topic'), (Image, 'depth_topic'),
                                            (CameraInfo, 'camera_info_topic'))]
            self.sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, self.p['sync_queue_size'], self.p['sync_slop_sec'])
            self.sync.registerCallback(lambda color, depth, info:
                                       self._receive(('rgbd', color, depth, info)))
        self.create_timer(self.p['processing_period_sec'], self._process)
        self.get_logger().info(
            f'Tabletop perception frame_mode={self.p["frame_mode"]}, outputs={self.namespace}; '
            'camera-facing plane normal is not a gravity measurement; no commands'
            if camera_mode else 'Tabletop perception: timestamped TF, no tag detector, no commands')

    def _receive(self, item):
        self.pending = item  # Bounded: no backlog of obsolete geometry.
        self.last_received = time.monotonic()

    def _process(self):
        if self.pending is None:
            if time.monotonic() - self.last_received > self.p['max_age_sec']:
                self._invalid('RGB-D/point-cloud stream missing or expired')
            return
        item, self.pending = self.pending, None
        header = item[1].header if item[0] == 'cloud' else item[2].header
        self.last_source_header = deepcopy(header)
        try:
            self._check_header(header)
            if item[0] == 'cloud':
                points = cloud_xyz(item[1])
                if self.p['cloud_is_optical']:
                    points = points[(points[:, 2] >= self.cfg.min_depth_m)
                                    & (points[:, 2] <= self.cfg.max_depth_m)]
            else:
                _, color, depth, info = item
                if len({msg.header.frame_id for msg in (color, depth, info)}) != 1:
                    raise ValueError('registered RGB-D and CameraInfo optical frames differ')
                if depth.encoding not in ('16UC1', '32FC1'):
                    raise ValueError(f'unsupported depth encoding {depth.encoding}')
                points = rgbd_to_xyz(self.bridge.imgmsg_to_cv2(color, 'rgb8'),
                                     self.bridge.imgmsg_to_cv2(depth, 'passthrough'),
                                     info, self.cfg, self.p['rgbd_rectified'],
                                     inverse_brown_conrady=self.p['rgbd_inverse_brown_conrady'])
            self.cloud_pubs['input'].publish(cloud_message(header, points))
            if self.p['frame_mode'] == 'base' and header.frame_id != self.p['base_frame']:
                transform = self.buffer.lookup_transform(self.p['base_frame'], header.frame_id,
                                                         Time.from_msg(header.stamp))
                points = transform_points(points, transform)
            output_header = deepcopy(header)
            if self.p['frame_mode'] == 'base':
                output_header.frame_id = self.p['base_frame']
            observation = self.pipeline.process(points, stamp_sec=Time.from_msg(header.stamp).nanoseconds / 1e9)
            self._check_header(header)  # Reject observations that expired during processing.
            self._publish(output_header, observation)
        except (ValueError, TypeError, RuntimeError, TransformException, CvBridgeError) as error:
            self._invalid(str(error), header)

    def _check_header(self, header):
        ns = Time.from_msg(header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - ns) / 1e9
        if not header.frame_id or ns <= 0 or not -self.p['future_tolerance_sec'] <= age <= \
                self.p['max_age_sec']:
            raise ValueError(f'invalid/stale/future observation header (age={age:.3f}s)')

    def _invalid(self, reason, source_header=None):
        self.pipeline.reset(expire=True)
        header = Header(frame_id=self.p['base_frame'], stamp=self.get_clock().now().to_msg())
        if source_header is not None:
            header.stamp = source_header.stamp
        if self.p['frame_mode'] == 'camera':
            source = source_header or self.last_source_header
            header.frame_id = source.frame_id if source is not None else ''
        self._publish(header, Observation(np.empty((0, 3)), reason=reason))

    def _publish(self, header, observation):
        self.scene_pub.publish(scene_message(header, observation))
        self.marker_pub.publish(observation_markers(
            header, observation, self.p['marker_lifetime_sec'], self.p['label_height_m']))
        if self.preview_pub is not None:
            self.preview_pub.publish(orientation_preview_markers(
                header, observation, self.p['marker_lifetime_sec'], self.p['label_height_m']))
        for name, points in (('filtered', observation.filtered), ('table', observation.table_points),
                             ('target', observation.target_points)):
            self.cloud_pubs[name].publish(cloud_message(header, points))
        status = DiagnosticStatus(name=f'{self.namespace.strip("/")}/perception', hardware_id='rgbd',
                                  level=DiagnosticStatus.OK if observation.valid else DiagnosticStatus.WARN,
                                  message=observation.reason or 'valid tabletop geometry')
        status.values = self._diagnostic_values(header, observation)
        self.diag_pub.publish(DiagnosticArray(header=header, status=[status]))
        if observation.reason != self.last_reason:
            self.get_logger().info(observation.reason or 'Validated tabletop object')
            self.last_reason = observation.reason
        if observation.valid:
            box = observation.boxes[observation.target_index]
            pose = pose_from_box(box)
            tf = TransformStamped(header=header, child_frame_id=self.p['object_frame'])
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            self.broadcaster.sendTransform(tf)

    def _diagnostic_values(self, header, observation):
        """Numbers and explicit frame provenance, never a calibration certificate."""
        camera_mode = self.p['frame_mode'] == 'camera'
        source = self.last_source_header
        values = {
            'processing_frame': header.frame_id,
            'source_frame': source.frame_id if source else '',
            'frame_mode': self.p['frame_mode'],
            'geometry_valid': observation.valid,
            'filtered_points': len(observation.filtered),
            'table_points': len(observation.table_points),
            'target_points': len(observation.target_points),
            'objects': len(observation.boxes),
            'table_inlier_ratio': observation.plane_ratio,
            'normal_semantics': ('camera-facing support-plane normal; gravity unverified'
                                 if camera_mode else 'normal toward base +Z'),
            'physical_table_identity_verified': False,
            'base_tf_available': False,
            'base_tf_trustworthy': False,
            'base_tf_trust_reason': 'physical extrinsics and current joint/TCP feedback not verified here',
            'physical_grasp_validated': False,
            'candidate_rejection_reason': observation.reason or (
                'orientation preview only; jaw opening, TCP, calibration and robot state require verification'),
            'orientation_preview_count': 2 if camera_mode and observation.valid else 0,
            'cluster_geometry_metrics': observation.geometry_metrics,
            'metrology_consistency': observation.metrology_metrics,
            'full_extent_verified': False,
            'measurement_kind': 'visible_top_support' if self.cfg.refine_top_surface else 'visible_depth_bounds',
        }
        if source is not None:
            age = (self.get_clock().now().nanoseconds - Time.from_msg(source.stamp).nanoseconds) / 1e9
            values['source_age_sec'] = age
            values['source_header_fresh'] = -self.p['future_tolerance_sec'] <= age <= self.p['max_age_sec']
        base_transform = None
        if source is not None and source.frame_id and source.frame_id != self.p['base_frame']:
            try:
                base_transform = self.buffer.lookup_transform(
                    self.p['base_frame'], source.frame_id, Time.from_msg(source.stamp))
                values['base_tf_available'] = True
                values['base_tf_status'] = 'timestamped TF lookup available; physical accuracy unverified'
            except TransformException as error:
                values['base_tf_status'] = str(error)
        elif source is not None and source.frame_id == self.p['base_frame']:
            values['base_tf_available'] = True
            values['base_tf_status'] = 'input already declares base frame; provenance unverified'
        else:
            values['base_tf_status'] = 'source frame unavailable'
        if observation.plane is not None:
            values['table_plane_coefficients'] = observation.plane.tolist()
            values['table_normal'] = observation.plane[:3].tolist()
            residuals = observation.table_points @ observation.plane[:3] + observation.plane[3]
            values['table_rmse_m'] = float(np.sqrt(np.mean(residuals ** 2)))
        if observation.boxes:
            index = observation.target_index if observation.target_index >= 0 else 0
            box = observation.boxes[index]
            values.update({
                'reported_object_index': index,
                'reported_object_is_selected': observation.target_index >= 0,
                'dimensions_m': box.dimensions.tolist(),
                'center_processing_m': box.center.tolist(),
                'long_axis': box.rotation[:, 0].tolist(),
                'short_axis': box.rotation[:, 1].tolist(),
                'normal_axis': box.rotation[:, 2].tolist(),
                'horizontal_aspect_ratio': float(box.dimensions[0] / box.dimensions[1]),
                'object_point_count': box.point_count,
                'support_heuristic': box.quality,
                'yaw_observable': bool(box.yaw_observable),
                'dimension_semantics': 'visible support bounds; bottom inferred at fitted support plane',
            })
            if box.observed_bounds is not None:
                values['dimension_semantics'] = 'supported cuboid top rectangle; full depth envelope retained separately; collision unverified'
                values['full_depth_envelope_dimensions_m'] = box.observed_bounds.dimensions.tolist()
                values['full_depth_envelope_center_m'] = box.observed_bounds.center.tolist()
                values['full_depth_envelope_axes_columns'] = box.observed_bounds.rotation.tolist()
            if camera_mode:
                values['center_camera_m'] = box.center.tolist()
                if base_transform is not None:
                    values['center_base_m'] = transform_points(box.center.reshape(1, 3), base_transform)[0].tolist()
            else:
                values['center_base_m'] = box.center.tolist()
                if source is not None and source.frame_id != self.p['base_frame']:
                    try:
                        camera_transform = self.buffer.lookup_transform(
                            source.frame_id, self.p['base_frame'], Time.from_msg(source.stamp))
                        values['center_camera_m'] = transform_points(
                            box.center.reshape(1, 3), camera_transform)[0].tolist()
                    except TransformException:
                        pass
        return [KeyValue(key=name, value=json.dumps(value, allow_nan=False))
                for name, value in values.items()]


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
