#!/usr/bin/env python3
"""Read-only real-scene evidence, including invalid observations and TF provenance.

Labels describe human-arranged scenes; they are never fed into perception as
known object angles. This script creates no publishers and calls no services.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
import cv2
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from tabletop_grasp_interfaces.msg import TabletopScene
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from tabletop_grasp.geometry import transform_points
from tabletop_grasp.ros_geometry import cloud_xyz


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def rotation(pose):
    q = pose.orientation
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


class Evidence(Node):
    def __init__(self, args):
        super().__init__('tabletop_camera_evidence')
        self.args = args
        self.started = time.monotonic()
        self.counts = Counter()
        self.receipts = {}
        self.arrays = {}
        self.array_stamps = {}
        self.sampled = {}
        self.samples = []
        args.output.mkdir(parents=True, exist_ok=True)
        self.sample_stream = (args.output / 'scene_samples.live.jsonl').open('w')
        self.diagnostic_stream = (args.output / 'diagnostics.jsonl').open('w')
        self.transforms = {}
        self.latest_diagnostics = []
        self.info = None
        self.joints = {}
        self.preview = None
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_owned = []
        prefix = args.topic_prefix.rstrip('/')
        self.subscribe(TabletopScene, prefix + '/scene', self.scene, 10)
        self.subscribe(DiagnosticArray, prefix + '/diagnostics', self.diagnostics, 10)
        self.subscribe(MarkerArray, prefix + '/orientation_preview', self.orientation_preview, 10)
        self.subscribe(MarkerArray, prefix + '/perception_markers',
                       lambda msg: self.count('perception_markers', msg), 10)
        for name in ('input', 'filtered', 'table', 'target'):
            self.subscribe(PointCloud2, prefix + '/' + name + '_cloud',
                           lambda msg, name=name: self.cloud(msg, name), qos_profile_sensor_data)
        camera = '/wrist_camera/wrist_d405'
        for name, suffix in (('color', '/color/image_rect_raw'),
                             ('depth', '/aligned_depth_to_color/image_raw')):
            self.subscribe(Image, camera + suffix,
                           lambda msg, name=name: self.image(msg, name), qos_profile_sensor_data)
        self.subscribe(CameraInfo, camera + '/color/camera_info', self.camera_info,
                       qos_profile_sensor_data)
        self.subscribe(JointState, '/joint_states', self.joint_state, qos_profile_sensor_data)
        self.subscribe(TFMessage, '/tf', lambda msg: self.tf(msg, False), 100)
        self.subscribe(TFMessage, '/tf_static', lambda msg: self.tf(msg, True),
                       QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def subscribe(self, kind, topic, callback, qos):
        self.subscriptions_owned.append(self.create_subscription(kind, topic, callback, qos))

    def count(self, name, msg):
        now = time.monotonic() - self.started
        self.counts[name] += 1
        self.receipts.setdefault(name, []).append(now)

    def due(self, name):
        now = time.monotonic()
        if now - self.sampled.get(name, -float('inf')) < self.args.sample_period:
            return False
        self.sampled[name] = now
        return True

    def cloud(self, msg, name):
        self.count(name + '_cloud', msg)
        if self.due(name):
            self.arrays[name + '_points'] = cloud_xyz(msg)
            self.array_stamps[name + '_points'] = {
                'frame': msg.header.frame_id, 'stamp': stamp_seconds(msg.header)}

    def image(self, msg, name):
        self.count(name, msg)
        if self.due(name):
            self.arrays[name] = self.bridge.imgmsg_to_cv2(
                msg, 'rgb8' if name == 'color' else 'passthrough').copy()
            self.array_stamps[name] = {'frame': msg.header.frame_id,
                                       'stamp': stamp_seconds(msg.header)}

    def camera_info(self, msg):
        self.count('camera_info', msg)
        self.info = {'k': list(msg.k), 'p': list(msg.p), 'r': list(msg.r), 'd': list(msg.d),
                     'model': msg.distortion_model, 'width': msg.width, 'height': msg.height,
                     'frame': msg.header.frame_id}

    def joint_state(self, msg):
        self.count('joint_states', msg)
        for name, value in zip(msg.name, msg.position):
            self.joints[name] = {'position': value, 'stamp': stamp_seconds(msg.header)}

    def tf(self, msg, static):
        for transform in msg.transforms:
            self.transforms[transform.child_frame_id] = {
                'parent': transform.header.frame_id, 'child': transform.child_frame_id,
                'static': static, 'stamp': stamp_seconds(transform.header),
                'translation_m': xyz(transform.transform.translation),
                'receipt_elapsed_sec': time.monotonic() - self.started,
                'physical_accuracy_verified': False}

    def diagnostics(self, msg):
        self.count('diagnostics', msg)
        self.latest_diagnostics = [
            {'name': item.name,
             'level': int.from_bytes(item.level, 'little') if isinstance(item.level, bytes) else item.level,
             'message': item.message,
             'values': {pair.key: pair.value for pair in item.values}}
            for item in msg.status]
        self.diagnostic_stream.write(json.dumps({
            'elapsed_sec': time.monotonic() - self.started,
            'stamp': stamp_seconds(msg.header), 'statuses': self.latest_diagnostics}) + '\n')
        self.diagnostic_stream.flush()

    def orientation_preview(self, msg):
        self.count('orientation_preview', msg)
        arrows = [marker for marker in msg.markers if marker.action == Marker.ADD
                  and marker.type == Marker.ARROW and len(marker.points) == 2]
        axes = []
        for arrow in arrows:
            axis = np.asarray(xyz(arrow.points[1])) - xyz(arrow.points[0])
            if np.linalg.norm(axis) > 0:
                axes.append((axis / np.linalg.norm(axis)).tolist())
        header = msg.markers[0].header if msg.markers else None
        self.preview = {'frame': header.frame_id if header else None, 'count': len(axes),
                        'stamp': stamp_seconds(header) if header else None,
                        'is_feasible_grasp': False, 'closing_axes': axes}

    def scene(self, msg):
        self.count('scene', msg)
        sample = {'elapsed_sec': time.monotonic() - self.started,
                  'stamp': stamp_seconds(msg.header), 'frame': msg.header.frame_id,
                  'valid': msg.valid, 'reason': msg.reason,
                  'table_point_count': msg.table_point_count,
                  'table_inlier_ratio': msg.table_inlier_ratio,
                  'object_count': len(msg.objects), 'target_index': msg.target_index,
                  'base_transform_available': False, 'base_transform_accuracy_verified': False}
        if msg.table_point_count:
            sample['table_normal'] = rotation(msg.table_pose)[:, 2].tolist()
        if msg.valid and 0 <= msg.target_index < len(msg.objects):
            obj = msg.objects[msg.target_index]
            axes = rotation(obj.pose)
            center = xyz(obj.pose.position)
            sample.update(dimensions_m=xyz(obj.dimensions), center_processing_m=center,
                          long_axis=axes[:, 0].tolist(), short_axis=axes[:, 1].tolist(),
                          target_point_count=obj.point_count, quality=obj.quality,
                          yaw_observable=obj.yaw_observable)
            for name, frame in (('camera', self.args.camera_frame), ('base', self.args.base_frame)):
                try:
                    if frame == msg.header.frame_id:
                        transformed = center
                    else:
                        tf = self.buffer.lookup_transform(frame, msg.header.frame_id,
                                                          Time.from_msg(msg.header.stamp))
                        transformed = transform_points(np.asarray([center]), tf)[0].tolist()
                    sample['center_' + name + '_m'] = transformed
                    if name == 'base':
                        sample['base_transform_available'] = True
                except TransformException as error:
                    sample['center_' + name + '_m'] = None
                    sample[name + '_tf_reason'] = str(error)
        self.samples.append(sample)
        self.sample_stream.write(json.dumps(sample) + '\n')
        self.sample_stream.flush()

    def save(self):
        self.sample_stream.close()
        self.diagnostic_stream.close()
        output = self.args.output
        output.mkdir(parents=True, exist_ok=True)
        elapsed = time.monotonic() - self.started
        valid = [sample for sample in self.samples if sample['valid'] and 'long_axis' in sample]
        steps = []
        for previous, current in zip(valid, valid[1:]):
            if current['stamp'] - previous['stamp'] > 1.5:
                continue
            dot = np.clip(np.dot(previous['long_axis'], current['long_axis']), -1., 1.)
            steps.append({'elapsed_sec': current['elapsed_sec'],
                          'signed_axis_change_deg': float(np.degrees(np.arccos(dot))),
                          'pi_equivalent_axis_change_deg': float(np.degrees(np.arccos(abs(dot))))})
        services = sorted(name for name, _ in self.get_service_names_and_types())
        summary = {'label': self.args.label, 'label_is_ground_truth_angle': False,
                   'mode': 'real_camera_observation_only', 'elapsed_sec': elapsed,
                   'topic_prefix': self.args.topic_prefix, 'counts': dict(self.counts),
                   'received_rates_hz': {key: value / elapsed for key, value in self.counts.items()},
                   'valid_observations': len(valid), 'total_observations': len(self.samples),
                   'invalid_reasons': dict(Counter(s['reason'] for s in self.samples if not s['valid'])),
                   'last_valid': valid[-1] if valid else None,
                   'orientation_steps': steps, 'orientation_preview': self.preview,
                   'camera_info': self.info, 'array_stamps': self.array_stamps,
                   'array_point_counts': {k: len(v) for k, v in self.arrays.items() if k.endswith('_points')},
                   'latest_diagnostics': self.latest_diagnostics,
                   'observed_transforms': list(self.transforms.values()),
                   'observed_joints': self.joints,
                   'nodes': sorted(self.get_node_names_and_namespaces()),
                   'command_publishers': {topic: len(self.get_publishers_info_by_topic(topic)) for topic in
                                          ['/arm/joint_command', '/gripper/pulse_us', '/arm/preview_joint_states']},
                   'execution_services': [name for name in services if
                                          '/execute_trajectory/' in name or '/move_action/' in name]}
        if valid:
            dimensions = np.array([s['dimensions_m'] for s in valid])
            summary['dimensions_mean_m'] = dimensions.mean(axis=0).tolist()
            summary['dimensions_std_m'] = dimensions.std(axis=0).tolist()
        (output / 'observation.json').write_text(json.dumps(summary, indent=2) + '\n')
        (output / 'scene_samples.jsonl').write_text(''.join(json.dumps(s) + '\n' for s in self.samples))
        np.savez_compressed(output / 'capture.npz', **self.arrays)
        if 'color' in self.arrays:
            cv2.imwrite(str(output / 'color.png'), cv2.cvtColor(self.arrays['color'], cv2.COLOR_RGB2BGR))
        print(json.dumps({key: summary[key] for key in
                          ['counts', 'valid_observations', 'invalid_reasons', 'last_valid',
                           'command_publishers', 'execution_services']}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--label', default='stationary_unmeasured')
    parser.add_argument('--topic-prefix', default='/tabletop_camera')
    parser.add_argument('--sample-period', type=float, default=1.)
    parser.add_argument('--camera-frame', default='wrist_d405_color_optical_frame')
    parser.add_argument('--base-frame', default='base_link')
    args = parser.parse_args()
    if not np.isfinite([args.seconds, args.sample_period]).all() or min(args.seconds, args.sample_period) <= 0:
        parser.error('duration and sample period must be finite and positive')
    # Keep the graph alive while saving after Ctrl-C; rclpy's default SIGINT
    # handler otherwise invalidates the context before the final inventory.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Evidence(args)
    try:
        while time.monotonic() - node.started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
