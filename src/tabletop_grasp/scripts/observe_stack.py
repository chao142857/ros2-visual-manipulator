#!/usr/bin/env python3
"""Read-only capture of a running stack. Never publishes or calls a service."""
import argparse
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import DisplayTrajectory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import String
from tabletop_grasp_interfaces.msg import TabletopScene
from visualization_msgs.msg import MarkerArray
from tabletop_grasp.ros_geometry import cloud_xyz

parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['synthetic', 'camera'], required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--seconds', type=float, default=35.)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = Node('tabletop_read_only_observer')
counts, summary, arrays = {}, {'mode': args.mode, 'states': [], 'rejections': []}, {}
bridge = CvBridge()
subscriptions = []


def count(name):
    counts[name] = counts.get(name, 0) + 1


def scene(msg):
    count('scene')
    summary['scene_valid'] = msg.valid
    summary['scene_reason'] = msg.reason
    if msg.valid:
        obj = msg.objects[msg.target_index]
        summary['object'] = {
            'dimensions_m': [obj.dimensions.x, obj.dimensions.y, obj.dimensions.z],
            'center_m': [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z],
            'quaternion_xyzw': [obj.pose.orientation.x, obj.pose.orientation.y,
                                obj.pose.orientation.z, obj.pose.orientation.w],
            'point_count': obj.point_count, 'yaw_observable': obj.yaw_observable,
            'table_inlier_ratio': msg.table_inlier_ratio,
        }


def status(msg):
    count('status')
    data = json.loads(msg.data)
    if not summary['states'] or summary['states'][-1] != data['state']:
        summary['states'].append(data['state'])
    summary['last_status'] = data
    summary['rejections'] = data['rejections']
    if data['state'] == 'READY_TO_EXECUTE':
        summary['ready_seen'] = True


def trajectory(msg):
    count('display_trajectory')
    if msg.trajectory:
        summary['trajectory_segments'] = len(msg.trajectory)
        summary['trajectory_points'] = [len(t.joint_trajectory.points) for t in msg.trajectory]
        summary['trajectory_joints'] = [list(t.joint_trajectory.joint_names) for t in msg.trajectory]
        summary['trajectory_start'] = dict(zip(msg.trajectory_start.joint_state.name,
                                               msg.trajectory_start.joint_state.position))


def points(msg):
    count('input_cloud')
    xyz = cloud_xyz(msg)
    summary['input_frame'] = msg.header.frame_id
    summary['input_point_count'] = len(xyz)
    if len(xyz):
        summary['input_xyz_min'] = xyz.min(axis=0).tolist()
        summary['input_xyz_max'] = xyz.max(axis=0).tolist()
        arrays['points'] = xyz


def image(msg, name):
    count(name)
    arrays[name] = bridge.imgmsg_to_cv2(msg, 'rgb8' if name == 'color' else 'passthrough')
    summary[name] = {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding,
                     'frame': msg.header.frame_id}


def info(msg):
    count('camera_info')
    summary['camera_info'] = {'k': list(msg.k), 'p': list(msg.p), 'r': list(msg.r),
                              'd': list(msg.d), 'model': msg.distortion_model,
                              'width': msg.width, 'height': msg.height, 'frame': msg.header.frame_id}


def candidates(msg):
    count('candidates')
    summary['candidate_count'] = len(msg.poses)


for kind, topic, callback, qos in (
    (TabletopScene, '/tabletop/scene', scene, 10),
    (String, '/tabletop/planner_status', status, 10),
    (DisplayTrajectory, '/tabletop/display_planned_path', trajectory, 10),
    (PoseArray, '/tabletop/grasp_candidates', candidates, 10),
    (PointCloud2, '/tabletop/input_cloud', points, qos_profile_sensor_data),
    (MarkerArray, '/tabletop/perception_markers', lambda msg: count('perception_markers'), 10),
    (MarkerArray, '/tabletop/grasp_markers', lambda msg: count('grasp_markers'), 10),
):
    subscriptions.append(node.create_subscription(kind, topic, callback, qos))
if args.mode == 'camera':
    prefix = '/wrist_camera/wrist_d405'
    for name, suffix in [('color', '/color/image_rect_raw'), ('depth', '/aligned_depth_to_color/image_raw')]:
        subscriptions.append(node.create_subscription(Image, prefix + suffix,
                             lambda msg, name=name: image(msg, name), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(CameraInfo, prefix + '/color/camera_info',
                         info, qos_profile_sensor_data))
start = time.monotonic()
while time.monotonic() - start < args.seconds:
    rclpy.spin_once(node, timeout_sec=0.1)
summary['counts'] = counts
summary['elapsed_sec'] = time.monotonic() - start
summary['nodes'] = sorted(node.get_node_names_and_namespaces())
summary['services'] = sorted(name for name, _ in node.get_service_names_and_types())
summary['command_publishers'] = {
    topic: len(node.get_publishers_info_by_topic(topic))
    for topic in ['/arm/joint_command', '/gripper/pulse_us', '/arm/preview_joint_states']}
summary['execution_services'] = [
    name for name in summary['services']
    if '/execute_trajectory/' in name or '/move_action/' in name]
(args.output / 'observation.json').write_text(json.dumps(summary, indent=2) + '\n')
if arrays:
    np.savez_compressed(args.output / 'capture.npz', **arrays)
if 'color' in arrays:
    cv2.imwrite(str(args.output / 'color.png'), cv2.cvtColor(arrays['color'], cv2.COLOR_RGB2BGR))
summary_keys = ['mode', 'scene_valid', 'scene_reason', 'object', 'last_status', 'ready_seen',
                'trajectory_segments', 'counts', 'command_publishers', 'execution_services']
print(json.dumps({key: summary.get(key) for key in summary_keys}, indent=2))
node.destroy_node()
rclpy.shutdown()
if args.mode == 'synthetic' and not summary.get('ready_seen'):
    raise SystemExit(2)
