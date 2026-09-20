"""Manual, timestamp-synchronised hand-eye sample capture.

The target detector publishes a :class:`geometry_msgs.msg.PoseStamped` whose
pose is ``T_camera_target`` and whose header stamp is the image observation
time. A capture request uses that exact stamp for the robot TF lookup, so a
sample contains a time-aligned ``T_base_tool`` and ``T_camera_target`` pair.

This node only records calibration data. It never sends an arm or gripper
command.
"""

from __future__ import annotations

import copy
import math
import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .hand_eye_calibration import Sample, save_samples


def _finite_vector(values, name: str) -> np.ndarray:
    """Return a finite one-dimensional vector or raise a useful error."""
    vector = np.asarray(values, dtype=float).reshape(-1)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f'{name} contains non-finite values')
    return vector


def _matrix_from_rotation_translation(rotation, translation, name: str):
    """Build a finite homogeneous matrix from ROS rotation/translation fields."""
    q = _finite_vector(
        (rotation.x, rotation.y, rotation.z, rotation.w), f'{name} quaternion'
    )
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-12:
        raise ValueError(f'{name} quaternion has zero norm')
    x, y, z, w = (float(value) for value in q / norm)
    vector = _finite_vector(
        (translation.x, translation.y, translation.z), f'{name} translation'
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
    matrix[:3, 3] = vector
    return matrix


def transform_to_matrix(transform):
    """Convert a ROS ``Transform`` message to a homogeneous matrix."""
    return _matrix_from_rotation_translation(
        transform.rotation, transform.translation, 'TF transform'
    )


def pose_to_matrix(pose):
    """Convert a ROS ``Pose`` message to ``T_camera_target``."""
    return _matrix_from_rotation_translation(
        pose.orientation, pose.position, 'target pose'
    )


def _stamp_nanoseconds(stamp) -> int:
    """Validate a ROS time stamp and return integer nanoseconds."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError('observation header has no valid sec/nanosec stamp') from error
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('observation timestamp is outside the ROS time range')
    value = sec * 1_000_000_000 + nanosec
    if value <= 0:
        raise ValueError('observation timestamp is zero')
    return value


def _normalise_frame(frame: str) -> str:
    """Normalise a ROS frame ID for comparison and metadata."""
    return str(frame or '').strip().strip('/')


def _as_bool(value) -> bool:
    """Parse ROS parameter booleans correctly even for launch substitutions."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def pose_delta(first, second):
    """Return translation (m) and rotation (deg) change between tool poses."""
    translation = float(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    relative = first[:3, :3].T @ second[:3, :3]
    angle = math.degrees(
        math.acos(float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))
    )
    return translation, angle


class HandEyeCapture(Node):
    """Capture synchronised robot/target pairs on an explicit service call."""

    def __init__(self):
        super().__init__('hand_eye_capture')
        parameters = (
            ('base_frame', 'base_link'),
            ('tool_frame', 'tool0'),
            ('camera_frame', 'wrist_d405_color_optical_frame'),
            ('target_frame', 'wrist_calibration_target_6'),
            ('target_pose_topic', '/wrist_camera/wrist_d405/calibration/target_pose'),
            ('output_file', '/tmp/d405_hand_eye_samples.yaml'),
            ('minimum_translation_change_m', 0.01),
            ('minimum_rotation_change_deg', 5.0),
            ('sample_count', 20),
            ('max_observation_age_sec', 1.0),
            ('future_observation_tolerance_sec', 0.05),
            ('tf_lookup_timeout_sec', 0.2),
            ('overwrite', False),
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

        value = self.get_parameter
        self.base_frame = _normalise_frame(value('base_frame').value)
        self.tool_frame = _normalise_frame(value('tool_frame').value)
        self.camera_frame = _normalise_frame(value('camera_frame').value)
        self.target_frame = _normalise_frame(value('target_frame').value)
        self.target_pose_topic = str(value('target_pose_topic').value).strip()
        self.output_file = str(value('output_file').value)
        self.min_translation = float(value('minimum_translation_change_m').value)
        self.min_rotation = float(value('minimum_rotation_change_deg').value)
        self.goal = int(value('sample_count').value)
        self.max_observation_age_sec = float(value('max_observation_age_sec').value)
        self.future_observation_tolerance_sec = float(
            value('future_observation_tolerance_sec').value
        )
        self.tf_lookup_timeout_sec = float(value('tf_lookup_timeout_sec').value)
        self.overwrite = _as_bool(value('overwrite').value)
        if (
            not self.base_frame
            or not self.tool_frame
            or not self.camera_frame
            or not self.target_frame
            or not self.target_pose_topic
        ):
            raise ValueError('frame names and target_pose_topic must be non-empty')
        if (
            self.goal < 3
            or not math.isfinite(self.min_translation)
            or not math.isfinite(self.min_rotation)
            or self.min_translation < 0.0
            or self.min_rotation < 0.0
            or not math.isfinite(self.max_observation_age_sec)
            or self.max_observation_age_sec <= 0.0
            or not math.isfinite(self.future_observation_tolerance_sec)
            or self.future_observation_tolerance_sec < 0.0
            or not math.isfinite(self.tf_lookup_timeout_sec)
            or self.tf_lookup_timeout_sec < 0.0
        ):
            raise ValueError('invalid hand-eye capture parameters')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.samples: list[Sample] = []
        self.last_tool: Optional[np.ndarray] = None
        self.latest_target_pose: Optional[PoseStamped] = None
        self._captured_timestamps_ns: set[int] = set()
        self.last_observation_timestamp_ns: Optional[int] = None
        self.capture_locked = False
        self.saved_file: Optional[str] = None
        self._capture_lock = threading.Lock()

        self.target_subscription = self.create_subscription(
            PoseStamped, self.target_pose_topic, self.target_pose_callback, 10
        )
        self.service = self.create_service(Trigger, '/hand_eye/capture', self.capture)
        self.get_logger().info(
            f'Call /hand_eye/capture after each distinct pose ({self.goal} samples); '
            f'T_{self.base_frame}_{self.tool_frame}, '
            f'T_{self.camera_frame}_{self.target_frame} from {self.target_pose_topic}'
        )

    def target_pose_callback(self, message: PoseStamped):
        """Remember the newest valid target observation for the next request."""
        try:
            stamp_ns = _stamp_nanoseconds(message.header.stamp)
            source_frame = _normalise_frame(message.header.frame_id)
            if not source_frame:
                raise ValueError('target pose frame_id is empty')
            if source_frame != self.camera_frame:
                raise ValueError(
                    f'target pose frame is {source_frame}, expected {self.camera_frame}'
                )
            pose_to_matrix(message.pose)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'Ignoring invalid target observation: {error}')
            return
        self.latest_target_pose = copy.deepcopy(message)
        self.get_logger().debug(
            f'Received target observation stamp={stamp_ns} frame={source_frame}'
        )

    pose_callback = target_pose_callback

    def _fresh_observation(self):
        """Return ``(pose, stamp_ns, T_camera_target, age_sec)`` after checks."""
        message = self.latest_target_pose
        if message is None:
            raise ValueError(f'no target observation received on {self.target_pose_topic}')
        stamp_ns = _stamp_nanoseconds(message.header.stamp)
        if stamp_ns in self._captured_timestamps_ns:
            raise ValueError(f'duplicate observation timestamp: {stamp_ns} ns')
        source_frame = _normalise_frame(message.header.frame_id)
        if source_frame != self.camera_frame:
            raise ValueError(
                f'target pose frame is {source_frame}, expected {self.camera_frame}'
            )
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns <= 0:
            raise ValueError('node clock has not produced a non-zero time')
        age_ns = now_ns - stamp_ns
        max_age_ns = int(self.max_observation_age_sec * 1_000_000_000)
        future_tolerance_ns = int(
            self.future_observation_tolerance_sec * 1_000_000_000
        )
        if age_ns > max_age_ns:
            raise ValueError(
                f'target observation is stale: age={age_ns / 1e9:.3f} s '
                f'(limit {self.max_observation_age_sec:.3f} s)'
            )
        if age_ns < -future_tolerance_ns:
            raise ValueError(
                f'target observation is from the future: offset={-age_ns / 1e9:.3f} s'
            )
        return message, stamp_ns, pose_to_matrix(message.pose), age_ns / 1e9

    def _duplicate_tool_pose(self, tool_matrix: np.ndarray):
        """Return the first captured pose that is too close to this one."""
        for sample in self.samples:
            translation, rotation = pose_delta(sample.T_base_tool, tool_matrix)
            exact = translation <= 1.0e-12 and rotation <= 1.0e-9
            within_thresholds = (
                translation < self.min_translation and rotation < self.min_rotation
            )
            if exact or within_thresholds:
                return translation, rotation
        return None

    def capture(self, _request, response):
        """Capture one pair, save at the goal, and then fail closed."""
        with self._capture_lock:
            if self.capture_locked:
                response.success = False
                response.message = (
                    f'capture locked after saving {self.saved_file or self.output_file}'
                )
                return response
            try:
                message, stamp_ns, camera_target, age_sec = self._fresh_observation()
                observation_time = Time.from_msg(message.header.stamp)
                base_tool = self.buffer.lookup_transform(
                    self.base_frame,
                    self.tool_frame,
                    observation_time,
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                )
                tool_matrix = transform_to_matrix(base_tool.transform)
                duplicate = self._duplicate_tool_pose(tool_matrix)
                if duplicate is not None:
                    translation, rotation = duplicate
                    response.success = False
                    response.message = (
                        f'duplicate pose rejected: {translation:.4f} m, {rotation:.2f} deg'
                    )
                    return response

                sample = Sample(
                    tool_matrix,
                    camera_target,
                    f'sample_{len(self.samples) + 1:03d}',
                    stamp_ns / 1e9,
                    {
                        'base_frame': self.base_frame,
                        'tool_frame': self.tool_frame,
                        'camera_frame': self.camera_frame,
                        'target_frame': self.target_frame,
                        'target_pose_topic': self.target_pose_topic,
                        'observation_stamp_ns': stamp_ns,
                        'observation_age_sec': age_sec,
                    },
                )
                candidate_samples = self.samples + [sample]
                if len(candidate_samples) >= self.goal:
                    saved_path = save_samples(
                        self.output_file, candidate_samples, overwrite=self.overwrite
                    )
                    self.samples.append(sample)
                    self.last_tool = tool_matrix
                    self._captured_timestamps_ns.add(stamp_ns)
                    self.last_observation_timestamp_ns = stamp_ns
                    self.saved_file = str(saved_path)
                    self.capture_locked = True
                    response.success = True
                    response.message = f'saved {len(self.samples)} samples: {self.saved_file}'
                    return response

                self.samples.append(sample)
                self.last_tool = tool_matrix
                self._captured_timestamps_ns.add(stamp_ns)
                self.last_observation_timestamp_ns = stamp_ns
                response.success = True
                response.message = f'captured {len(self.samples)}/{self.goal}'
            except (TransformException, ValueError, OSError, TypeError, RuntimeError) as error:
                response.success = False
                response.message = f'capture failed: {error}'
            return response


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    'HandEyeCapture',
    '_stamp_nanoseconds',
    'main',
    'pose_delta',
    'pose_to_matrix',
    'transform_to_matrix',
]

