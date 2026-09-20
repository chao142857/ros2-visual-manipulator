"""Transform generic stamped camera target points into a robot frame."""

from collections import deque
import math

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def point_is_finite(point):
    """Return whether all three Point coordinates are finite."""
    return all(math.isfinite(value) for value in (point.x, point.y, point.z))


def transform_point_stamped(
    tf_buffer,
    message,
    target_frame,
    timeout_sec,
    use_latest_transform=False,
):
    """Transform a PointStamped through tf2."""

    if use_latest_transform:
        lookup_time = Time()
    else:
        lookup_time = Time.from_msg(
            message.header.stamp
        )

    transform = tf_buffer.lookup_transform(
        target_frame,
        message.header.frame_id.strip(),
        lookup_time,
        timeout=Duration(
            seconds=timeout_sec
        ),
    )

    transformed_point = do_transform_point(
        message,
        transform
    )

    # 保留目标实际观测时间
    transformed_point.header.stamp = (
        message.header.stamp
    )

    transformed_point.header.frame_id = (
        target_frame
    )

    return transformed_point


class TargetTransformNode(Node):
    """Transform incoming PointStamped targets through tf2."""

    def __init__(self):
        """Create the generic target transformer and optional marker output."""
        super().__init__('target_transform_node')

        self.declare_parameter(
            'input_topic', '/perception/target_point_camera'
        )
        self.declare_parameter(
            'output_topic', '/perception/target_point_base'
        )
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('max_message_age_sec', 1.0)
        self.declare_parameter('future_tolerance_sec', 0.1)
        self.declare_parameter('transform_timeout_sec', 0.0)
        self.declare_parameter('log_throttle_sec', 5.0)
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('marker_topic', '/perception/target_marker')
        self.declare_parameter('marker_scale_m', 0.04)
        self.declare_parameter('marker_lifetime_sec', 0.5)
        self.declare_parameter(
            'use_latest_transform',
            False
        )

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value.strip()
        self.max_message_age_sec = self.get_parameter(
            'max_message_age_sec'
        ).value
        self.future_tolerance_sec = self.get_parameter(
            'future_tolerance_sec'
        ).value
        self.transform_timeout_sec = self.get_parameter(
            'transform_timeout_sec'
        ).value
        self.log_throttle_sec = self.get_parameter('log_throttle_sec').value
        self.publish_marker = self.get_parameter('publish_marker').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.marker_scale_m = self.get_parameter('marker_scale_m').value
        self.marker_lifetime_sec = self.get_parameter(
            'marker_lifetime_sec'
        ).value
        self.use_latest_transform = (
            self.get_parameter(
                'use_latest_transform'
            ).value
        )
        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Camera observations commonly arrive a few milliseconds before the
        # matching /tf sample. A synchronous lookup inside the subscription
        # callback prevents a single-threaded executor from receiving that TF,
        # so retain a short bounded history and retry from a timer instead.
        self.pending_points = deque(maxlen=32)
        self.last_transform_error = None
        self.point_publisher = self.create_publisher(
            PointStamped, self.output_topic, 10
        )
        self.point_subscription = self.create_subscription(
            PointStamped, self.input_topic, self.point_callback, 10
        )
        self.retry_timer = self.create_timer(
            0.01,
            self._process_pending_points,
        )
        self.marker_publisher = None
        if self.publish_marker:
            self.marker_publisher = self.create_publisher(
                Marker, self.marker_topic, 10
            )

        self.get_logger().info(
            'Target Point Transformer started\n'
            f'  input        = {self.input_topic}\n'
            f'  output       = {self.output_topic}\n'
            f'  target frame = {self.target_frame}\n'
            f'  latest TF    = '
            f'{self.use_latest_transform}\n'
            f'  marker       = {self.publish_marker}'
        )

    def _validate_parameters(self):
        """Reject invalid configuration before creating ROS interfaces."""
        numeric_parameters = {
            'max_message_age_sec': self.max_message_age_sec,
            'future_tolerance_sec': self.future_tolerance_sec,
            'transform_timeout_sec': self.transform_timeout_sec,
            'log_throttle_sec': self.log_throttle_sec,
            'marker_scale_m': self.marker_scale_m,
            'marker_lifetime_sec': self.marker_lifetime_sec,
        }
        for name, value in numeric_parameters.items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not self.input_topic or not self.output_topic:
            raise ValueError('input_topic and output_topic must be non-empty')
        if not self.target_frame:
            raise ValueError('target_frame must be non-empty')
        if self.max_message_age_sec < 0.0:
            raise ValueError('max_message_age_sec must be non-negative')
        if self.future_tolerance_sec < 0.0:
            raise ValueError('future_tolerance_sec must be non-negative')
        if self.transform_timeout_sec < 0.0:
            raise ValueError('transform_timeout_sec must be non-negative')
        if self.log_throttle_sec <= 0.0:
            raise ValueError('log_throttle_sec must be positive')
        if self.publish_marker and not self.marker_topic:
            raise ValueError('marker_topic must be non-empty when enabled')
        if self.marker_scale_m <= 0.0:
            raise ValueError('marker_scale_m must be positive')
        if self.marker_lifetime_sec < 0.0:
            raise ValueError('marker_lifetime_sec must be non-negative')

    def _warn_throttled(self, message):
        """Warn at a bounded rate for bad data or unavailable transforms."""
        self.get_logger().warning(
            message,
            throttle_duration_sec=self.log_throttle_sec,
        )

    def point_callback(self, message):
        """Validate, transform, and publish one target observation."""
        source_frame = message.header.frame_id.strip()
        if not source_frame:
            self._warn_throttled(
                'Dropping target point with an empty frame_id'
            )
            return
        if not point_is_finite(message.point):
            self._warn_throttled(
                'Dropping target point containing NaN or infinity'
            )
            return

        has_zero_stamp = (
            message.header.stamp.sec == 0
            and message.header.stamp.nanosec == 0
        )

        if (
            has_zero_stamp
            and not self.use_latest_transform
        ):
            self._warn_throttled(
                'Dropping target point with an '
                'uninitialized timestamp'
            )
            return

        if not has_zero_stamp:

            try:
                source_time = Time.from_msg(
                    message.header.stamp
                )

            except (
                OverflowError,
                TypeError,
                ValueError
            ) as error:

                self._warn_throttled(
                    f'Dropping target with '
                    f'invalid timestamp: {error}'
                )

                return


            age_sec = (
                self.get_clock().now().nanoseconds
                - source_time.nanoseconds
            ) / 1.0e9


            if (
                self.max_message_age_sec > 0.0
                and age_sec > self.max_message_age_sec
            ):

                self._warn_throttled(
                    'Dropping stale target point '
                    f'(age={age_sec:.3f} s)'
                )

                return


            if (
                age_sec
                < -self.future_tolerance_sec
            ):

                self._warn_throttled(
                    'Dropping target point too far '
                    'in the future '
                    f'(offset={-age_sec:.3f} s)'
                )

                return

        self.pending_points.append(message)
        self._process_pending_points()

    def _process_pending_points(self):
        """Publish queued observations once their timestamped TF is ready."""
        processed = 0
        while self.pending_points and processed < 8:
            message = self.pending_points[0]
            if not self.use_latest_transform:
                source_time = Time.from_msg(message.header.stamp)
                age_sec = (
                    self.get_clock().now().nanoseconds
                    - source_time.nanoseconds
                ) / 1.0e9
                if (
                    self.max_message_age_sec > 0.0
                    and age_sec > self.max_message_age_sec
                ):
                    self.pending_points.popleft()
                    self._warn_throttled(
                        'Dropping target after timestamped TF did not arrive '
                        f'(age={age_sec:.3f} s, error='
                        f'{self.last_transform_error})'
                    )
                    continue

            try:
                transformed_point = transform_point_stamped(
                    self.tf_buffer,
                    message,
                    self.target_frame,
                    # Never block the executor here: the timer retry lets TF
                    # subscriptions run between attempts.
                    0.0,
                    use_latest_transform=self.use_latest_transform,
                )
            except TransformException as error:
                self.last_transform_error = error
                break

            self.pending_points.popleft()
            self.last_transform_error = None
            self._publish_transformed_point(transformed_point)
            processed += 1

    def _publish_transformed_point(self, transformed_point):
        """Publish one already transformed finite point and optional marker."""

        if not point_is_finite(transformed_point.point):
            self._warn_throttled(
                'Dropping target because tf2 produced non-finite coordinates'
            )
            return

        self.point_publisher.publish(transformed_point)

        if self.marker_publisher is not None:
            self.marker_publisher.publish(
                self._make_marker(transformed_point)
            )

    def _make_marker(self, transformed_point):
        """Create a short-lived sphere marker at the transformed target."""
        marker = Marker()
        marker.header = transformed_point.header
        marker.ns = 'target_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = transformed_point.point.x
        marker.pose.position.y = transformed_point.point.y
        marker.pose.position.z = transformed_point.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale_m
        marker.scale.y = self.marker_scale_m
        marker.scale.z = self.marker_scale_m
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.05
        marker.color.a = 0.95
        marker.lifetime = Duration(
            seconds=self.marker_lifetime_sec
        ).to_msg()
        return marker


def main(args=None):
    """Run the target transformer."""
    rclpy.init(args=args)
    node = TargetTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
