"""Transform stamped calibration poses into the robot planning frame with tf2."""

from __future__ import annotations

from collections import deque
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


def pose_is_finite(message):
    """Return whether the pose contains finite position and orientation."""
    values = (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    )
    return all(math.isfinite(float(value)) for value in values)


def transform_pose_stamped(
    tf_buffer, message, target_frame, timeout_sec=0.0
):
    """Transform both position and orientation at the observation timestamp."""
    transform = tf_buffer.lookup_transform(
        target_frame,
        message.header.frame_id.strip(),
        Time.from_msg(message.header.stamp),
        timeout=Duration(seconds=timeout_sec),
    )
    output = do_transform_pose_stamped(message, transform)
    output.header.frame_id = target_frame
    output.header.stamp = message.header.stamp
    return output


class TargetPoseTransformNode(Node):
    """Retry timestamped TF lookups until AprilTag poses reach base_link."""

    def __init__(self):
        super().__init__('target_pose_transform_node')
        self.declare_parameter('input_topic', '/perception/target_pose_camera')
        self.declare_parameter('output_topic', '/perception/target_pose_base')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('max_message_age_sec', 1.0)
        self.declare_parameter('future_tolerance_sec', 0.1)
        self.declare_parameter('log_throttle_sec', 5.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(
            self.get_parameter('target_frame').value
        ).strip()
        self.max_message_age_sec = float(
            self.get_parameter('max_message_age_sec').value
        )
        self.future_tolerance_sec = float(
            self.get_parameter('future_tolerance_sec').value
        )
        self.log_throttle_sec = float(
            self.get_parameter('log_throttle_sec').value
        )
        if (
            not self.input_topic
            or not self.output_topic
            or not self.target_frame
        ):
            raise ValueError(
                'input_topic, output_topic, and target_frame are required'
            )
        if (
            not math.isfinite(self.max_message_age_sec)
            or not math.isfinite(self.future_tolerance_sec)
            or not math.isfinite(self.log_throttle_sec)
            or self.max_message_age_sec <= 0.0
            or self.future_tolerance_sec < 0.0
            or self.log_throttle_sec <= 0.0
        ):
            raise ValueError('pose-transform timing parameters are invalid')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_poses = deque(maxlen=32)
        self.publisher = self.create_publisher(
            PoseStamped, self.output_topic, 10
        )
        self.subscription = self.create_subscription(
            PoseStamped, self.input_topic, self.pose_callback, 10
        )
        self.timer = self.create_timer(0.01, self._process_pending_poses)
        self.last_transform_error = None
        self.get_logger().info(
            'Target Pose Transformer started\n'
            f'  input        = {self.input_topic}\n'
            f'  output       = {self.output_topic}\n'
            f'  target frame = {self.target_frame}'
        )

    def _warn(self, message):
        self.get_logger().warning(
            message, throttle_duration_sec=self.log_throttle_sec
        )

    def pose_callback(self, message):
        if not message.header.frame_id.strip() or not pose_is_finite(message):
            self._warn('Dropping AprilTag pose with invalid frame or values')
            return
        if (
            message.header.stamp.sec == 0
            and message.header.stamp.nanosec == 0
        ):
            self._warn(
                'Dropping AprilTag pose with an uninitialized timestamp'
            )
            return
        try:
            source_time = Time.from_msg(message.header.stamp)
        except (OverflowError, TypeError, ValueError) as error:
            self._warn(
                f'Dropping AprilTag pose with invalid timestamp: {error}'
            )
            return
        age_sec = (
            self.get_clock().now().nanoseconds - source_time.nanoseconds
        ) / 1.0e9
        if (
            age_sec > self.max_message_age_sec
            or age_sec < -self.future_tolerance_sec
        ):
            self._warn(
                f'Dropping stale/future AprilTag pose (age={age_sec:.3f} s)'
            )
            return
        self.pending_poses.append(message)
        self._process_pending_poses()

    def _process_pending_poses(self):
        while self.pending_poses:
            message = self.pending_poses[0]
            age_sec = (
                self.get_clock().now().nanoseconds
                - Time.from_msg(message.header.stamp).nanoseconds
            ) / 1.0e9
            if age_sec > self.max_message_age_sec:
                self.pending_poses.popleft()
                self._warn(
                    'Dropping AprilTag pose after timestamped TF did not '
                    'arrive '
                    f'(error={self.last_transform_error})'
                )
                continue
            try:
                transformed = transform_pose_stamped(
                    self.tf_buffer, message, self.target_frame
                )
            except (TransformException, ValueError) as error:
                self.last_transform_error = error
                return
            self.pending_poses.popleft()
            self.last_transform_error = None
            if pose_is_finite(transformed):
                self.publisher.publish(transformed)
            else:
                self._warn(
                    'Dropping AprilTag pose because TF produced invalid values'
                )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    'TargetPoseTransformNode',
    'main',
    'pose_is_finite',
    'transform_pose_stamped',
]
