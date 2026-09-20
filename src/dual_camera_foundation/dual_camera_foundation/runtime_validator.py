"""Bounded runtime validation for the Astra click-to-grasp preview chain."""

from collections import deque
from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

PASS = 'PASS'
WARN = 'WARN'
FAIL = 'FAIL'

JOINT_NAMES = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
)

DEFAULT_JOINT_LOWER_LIMITS = (-3.14,) * len(JOINT_NAMES)
DEFAULT_JOINT_UPPER_LIMITS = (3.14,) * len(JOINT_NAMES)
VALIDATION_LEVELS = ('startup', 'click', 'ik')

COMMAND_TOPICS = (
    '/arm/joint_command',
    '/arm/live_joint_command',
    '/gripper/pulse_us',
)


@dataclass(frozen=True)
class CheckResult:
    """One stable, printable validation result."""

    status: str
    detail: str

    @property
    def failed(self):
        """Return whether this result must fail the validation run."""
        return self.status == FAIL


@dataclass(frozen=True)
class StreamMetadata:
    """Small image or CameraInfo snapshot without retaining image data."""

    received_at: float
    width: int
    height: int
    frame_id: str
    encoding: str = ''


def estimate_rate(received_at, now=None, window_sec=None):
    """Estimate callback frequency from monotonic receive timestamps."""
    stamps = tuple(float(value) for value in received_at)
    if now is not None and window_sec is not None:
        lower_bound = float(now) - float(window_sec)
        stamps = tuple(value for value in stamps if value >= lower_bound)
    if len(stamps) < 2:
        return 0.0
    elapsed = stamps[-1] - stamps[0]
    if not math.isfinite(elapsed) or elapsed <= 0.0:
        return 0.0
    return (len(stamps) - 1) / elapsed


def evaluate_rate(
    name,
    received_at,
    now,
    window_sec,
    expected_fps,
    minimum_fps_ratio,
    freshness_sec,
):
    """Evaluate callback frequency and freshness for one live stream."""
    stamps = tuple(float(value) for value in received_at)
    if not stamps:
        return CheckResult(FAIL, f'{name}: no messages received')
    age_sec = float(now) - stamps[-1]
    if not math.isfinite(age_sec) or age_sec > freshness_sec:
        return CheckResult(
            FAIL,
            f'{name}: latest message is stale ({age_sec:.3f} s)',
        )
    actual_fps = estimate_rate(stamps, now, window_sec)
    minimum_fps = expected_fps * minimum_fps_ratio
    detail = (
        f'{name}: {actual_fps:.2f} Hz '
        f'(required >= {minimum_fps:.2f} Hz)'
    )
    if actual_fps < minimum_fps:
        return CheckResult(FAIL, detail)
    return CheckResult(PASS, detail)


def evaluate_registered_streams(
    rgb,
    depth,
    camera_info,
    expected_width,
    expected_height,
    expected_frame,
    now,
    freshness_sec,
):
    """Check registered RGB, depth, and intrinsics pixel geometry."""
    if rgb is None or depth is None or camera_info is None:
        missing = []
        if rgb is None:
            missing.append('RGB')
        if depth is None:
            missing.append('depth')
        if camera_info is None:
            missing.append('CameraInfo')
        return CheckResult(FAIL, 'missing ' + ', '.join(missing))

    samples = (rgb, depth, camera_info)
    stale = [
        float(now) - sample.received_at
        for sample in samples
        if float(now) - sample.received_at > freshness_sec
    ]
    if stale:
        return CheckResult(
            FAIL,
            f'RGB-D metadata is stale (max age={max(stale):.3f} s)',
        )

    sizes = tuple((sample.width, sample.height) for sample in samples)
    expected_size = (int(expected_width), int(expected_height))
    if any(size != expected_size for size in sizes):
        return CheckResult(
            FAIL,
            f'size mismatch: RGB={sizes[0]}, depth={sizes[1]}, '
            f'info={sizes[2]}, expected={expected_size}',
        )

    frames = tuple(sample.frame_id.strip() for sample in samples)
    if not all(frames) or len(set(frames)) != 1:
        return CheckResult(
            FAIL,
            f'frame mismatch: RGB={frames[0]!r}, depth={frames[1]!r}, '
            f'info={frames[2]!r}',
        )
    if frames[0] != str(expected_frame).strip():
        return CheckResult(
            FAIL,
            f'expected registered frame {expected_frame!r}, got '
            f'{frames[0]!r}',
        )
    if rgb.encoding not in ('bgr8', 'rgb8'):
        return CheckResult(FAIL, f'unexpected RGB encoding {rgb.encoding!r}')
    if depth.encoding not in ('16UC1', '32FC1'):
        return CheckResult(
            FAIL,
            f'unexpected depth encoding {depth.encoding!r}',
        )
    return CheckResult(
        PASS,
        f'registered {expected_size[0]}x{expected_size[1]} '
        f'in {frames[0]}',
    )


def normalize_node_name(node_name, node_namespace='/'):
    """Return one fully-qualified ROS node name."""
    name = str(node_name).strip('/')
    namespace = str(node_namespace).strip('/')
    if namespace:
        return f'/{namespace}/{name}'
    return f'/{name}'


def evaluate_command_publishers(
    publishers_by_topic,
    require_zero=True,
    forbidden_owner='/visual_grasp_controller',
):
    """Reject hardware-command publishers in a preview-only graph."""
    active = {
        topic: tuple(sorted(set(publishers_by_topic.get(topic, ()))))
        for topic in COMMAND_TOPICS
        if publishers_by_topic.get(topic, ())
    }
    forbidden_owner = normalize_node_name(forbidden_owner)
    owned_by_visual_grasp = {
        topic: owners
        for topic, owners in active.items()
        if forbidden_owner in owners
    }
    if owned_by_visual_grasp:
        return CheckResult(
            FAIL,
            'visual_grasp_controller owns command publisher(s): '
            f'{owned_by_visual_grasp}',
        )
    if active and require_zero:
        return CheckResult(
            FAIL,
            f'preview graph contains command publisher(s): {active}',
        )
    if active:
        return CheckResult(
            WARN,
            f'non-visual-grasp command publisher(s) present: {active}',
        )
    return CheckResult(PASS, 'no arm or gripper command publishers exist')


def evaluate_topic_publishers(
    topic,
    endpoints,
    expected_type,
    expected_owner,
    require_exactly_one=True,
):
    """Validate publisher type and owner while handling vendor endpoints."""
    endpoints = tuple(
        (str(topic_type), normalize_node_name(owner))
        for topic_type, owner in endpoints
    )
    if not endpoints:
        return CheckResult(FAIL, f'{topic} has no publisher')
    if require_exactly_one and len(endpoints) != 1:
        return CheckResult(
            FAIL,
            f'{topic} has {len(endpoints)} publishers, expected exactly 1',
        )
    wrong_types = sorted({
        topic_type
        for topic_type, _owner in endpoints
        if topic_type != expected_type
    })
    if wrong_types:
        return CheckResult(
            FAIL,
            f'{topic} has unexpected publisher types {wrong_types}; '
            f'expected {expected_type!r}',
        )
    expected_owner = normalize_node_name(expected_owner)
    owners = sorted({owner for _topic_type, owner in endpoints})
    if owners != [expected_owner]:
        return CheckResult(
            FAIL,
            f'{topic} owners={owners!r}, expected only {expected_owner!r}',
        )
    return CheckResult(
        PASS,
        f'{topic} has {len(endpoints)} endpoint(s) from {expected_owner}',
    )


def validate_joint_solution(
    names,
    positions,
    expected_names=JOINT_NAMES,
    lower_limits=DEFAULT_JOINT_LOWER_LIMITS,
    upper_limits=DEFAULT_JOINT_UPPER_LIMITS,
):
    """Validate a complete, finite, ordered and in-limit joint solution."""
    names = tuple(str(name) for name in names)
    expected_names = tuple(str(name) for name in expected_names)
    if names != expected_names:
        return CheckResult(
            FAIL,
            f'joint names {names!r} do not match {expected_names!r}',
        )
    if len(positions) != len(expected_names):
        return CheckResult(
            FAIL,
            f'expected {len(expected_names)} positions, got {len(positions)}',
        )
    values = tuple(float(value) for value in positions)
    if not all(math.isfinite(value) for value in values):
        return CheckResult(FAIL, 'joint positions contain NaN or infinity')
    lower = tuple(float(value) for value in lower_limits)
    upper = tuple(float(value) for value in upper_limits)
    if len(lower) != len(values) or len(upper) != len(values):
        return CheckResult(FAIL, 'joint-limit vector length mismatch')
    if not all(math.isfinite(value) for value in lower + upper):
        return CheckResult(FAIL, 'joint limits contain NaN or infinity')
    if any(low > high for low, high in zip(lower, upper)):
        return CheckResult(FAIL, 'joint lower limit exceeds upper limit')
    violations = [
        name
        for name, value, low, high in zip(names, values, lower, upper)
        if value < low - 1.0e-9 or value > high + 1.0e-9
    ]
    if violations:
        return CheckResult(
            FAIL,
            f'joint solution violates limits: {violations}',
        )
    return CheckResult(
        PASS,
        f'finite in-limit {len(values)}-joint solution',
    )


def transform_xyz(point_xyz, translation_xyz, quaternion_xyzw):
    """Apply a normalized rigid transform to a three-dimensional point."""
    point = tuple(float(value) for value in point_xyz)
    translation = tuple(float(value) for value in translation_xyz)
    quaternion = tuple(float(value) for value in quaternion_xyzw)
    if len(point) != 3 or len(translation) != 3 or len(quaternion) != 4:
        raise ValueError(
            'point, translation, and quaternion shapes are invalid'
        )
    values = point + translation + quaternion
    if not all(math.isfinite(value) for value in values):
        raise ValueError('transform inputs must be finite')
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm must be positive')
    qx, qy, qz, qw = (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm,
    )
    px, py, pz = point
    rx = (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * px
        + 2.0 * (qx * qy - qz * qw) * py
        + 2.0 * (qx * qz + qy * qw) * pz
    )
    ry = (
        2.0 * (qx * qy + qz * qw) * px
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * py
        + 2.0 * (qy * qz - qx * qw) * pz
    )
    rz = (
        2.0 * (qx * qz - qy * qw) * px
        + 2.0 * (qy * qz + qx * qw) * py
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * pz
    )
    return (
        rx + translation[0],
        ry + translation[1],
        rz + translation[2],
    )


def point_distance(left, right):
    """Return Euclidean distance between two finite XYZ triples."""
    left = tuple(float(value) for value in left)
    right = tuple(float(value) for value in right)
    if len(left) != 3 or len(right) != 3:
        raise ValueError('points must contain exactly three coordinates')
    if not all(math.isfinite(value) for value in left + right):
        return float('inf')
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def _stamp_key(message):
    stamp = message.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


def _point_xyz(message):
    return (
        float(message.point.x),
        float(message.point.y),
        float(message.point.z),
    )


class AstraClickGraspRuntimeValidator(Node):
    """Validate startup, one manual click, and dry-run IK in one process."""

    def __init__(self):
        super().__init__('astra_click_grasp_validator')

        self.declare_parameter('validation_level', 'ik')
        self.declare_parameter('startup_timeout_sec', 15.0)
        self.declare_parameter('click_timeout_sec', 60.0)
        self.declare_parameter('ik_timeout_sec', 15.0)
        self.declare_parameter('rate_window_sec', 3.0)
        self.declare_parameter('freshness_sec', 1.0)
        self.declare_parameter('expected_width', 640)
        self.declare_parameter('expected_height', 480)
        self.declare_parameter('expected_fps', 30.0)
        self.declare_parameter('minimum_fps_ratio', 0.7)
        self.declare_parameter('joint_state_min_fps', 5.0)
        self.declare_parameter(
            'expected_registered_frame',
            'camera_color_optical_frame',
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('required_target_pairs', 3)
        self.declare_parameter('required_preview_solutions', 6)
        self.declare_parameter('transform_tolerance_m', 0.002)
        self.declare_parameter('require_zero_command_publishers', True)
        self.declare_parameter(
            'required_nodes',
            [
                '/robot_state_publisher',
                '/camera/camera',
                '/arm_driver_node',
                '/camera_extrinsics_publisher_node',
                '/click_target_node',
                '/astra_click_target_transform_node',
                '/visual_grasp_controller',
            ],
        )
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/depth/camera_info',
        )
        self.declare_parameter('object_topic', '/camera/object_point')
        self.declare_parameter(
            'base_target_topic',
            '/perception/target_point_base',
        )
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter(
            'preview_topic',
            '/perception/dry_run/visual_grasp_solution',
        )

        self.validation_level = str(
            self.get_parameter('validation_level').value
        ).strip().lower()
        if self.validation_level not in VALIDATION_LEVELS:
            raise ValueError(
                'validation_level must be startup, click, or ik'
            )

        self.startup_timeout_sec = self._positive_float(
            'startup_timeout_sec'
        )
        self.click_timeout_sec = self._positive_float('click_timeout_sec')
        self.ik_timeout_sec = self._positive_float('ik_timeout_sec')
        self.rate_window_sec = self._positive_float('rate_window_sec')
        self.freshness_sec = self._positive_float('freshness_sec')
        self.expected_fps = self._positive_float('expected_fps')
        self.joint_state_min_fps = self._positive_float(
            'joint_state_min_fps'
        )
        self.transform_tolerance_m = self._positive_float(
            'transform_tolerance_m'
        )
        self.minimum_fps_ratio = float(
            self.get_parameter('minimum_fps_ratio').value
        )
        if not 0.0 < self.minimum_fps_ratio <= 1.0:
            raise ValueError('minimum_fps_ratio must be in (0, 1]')

        self.expected_width = int(
            self.get_parameter('expected_width').value
        )
        self.expected_height = int(
            self.get_parameter('expected_height').value
        )
        self.required_target_pairs = int(
            self.get_parameter('required_target_pairs').value
        )
        self.required_preview_solutions = int(
            self.get_parameter('required_preview_solutions').value
        )
        positive_integers = (
            self.expected_width,
            self.expected_height,
            self.required_target_pairs,
            self.required_preview_solutions,
        )
        if any(value <= 0 for value in positive_integers):
            raise ValueError('dimensions and required counts must be positive')

        self.expected_frame = str(
            self.get_parameter('expected_registered_frame').value
        ).strip()
        self.base_frame = str(
            self.get_parameter('base_frame').value
        ).strip()
        if not self.expected_frame or not self.base_frame:
            raise ValueError(
                'expected_registered_frame and base_frame required'
            )
        self.require_zero_command_publishers = bool(
            self.get_parameter('require_zero_command_publishers').value
        )
        self.required_nodes = tuple(
            normalize_node_name(value)
            for value in self.get_parameter('required_nodes').value
        )

        topic_parameter_names = (
            'rgb_topic',
            'depth_topic',
            'camera_info_topic',
            'object_topic',
            'base_target_topic',
            'joint_state_topic',
            'preview_topic',
        )
        self.topics = {
            name: str(self.get_parameter(name).value).strip()
            for name in topic_parameter_names
        }
        if not all(self.topics.values()):
            raise ValueError('all validation topics must be non-empty')

        self.rgb = None
        self.depth = None
        self.camera_info = None
        self.rgb_times = deque(maxlen=1000)
        self.depth_times = deque(maxlen=1000)
        self.joint_times = deque(maxlen=1000)
        self.latest_joint_result = CheckResult(
            FAIL,
            'no joint state received',
        )

        self.object_messages = {}
        self.base_messages = {}
        self.object_order = deque()
        self.base_order = deque()
        self.matched_stamps = set()
        self.valid_object_count = 0
        self.valid_base_count = 0
        self.invalid_object_reason = 'no clicked camera point received'
        self.invalid_base_reason = 'no transformed base point received'
        self.transform_errors = deque(maxlen=100)

        self.preview_message_count = 0
        self.preview_valid_count = 0
        self.latest_preview_at = None
        self.latest_preview_result = CheckResult(
            FAIL,
            'no dry-run IK solution received',
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image,
            self.topics['rgb_topic'],
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.topics['depth_topic'],
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.topics['camera_info_topic'],
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self.topics['object_topic'],
            self._object_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self.topics['base_target_topic'],
            self._base_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.topics['joint_state_topic'],
            self._joint_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.topics['preview_topic'],
            self._preview_callback,
            qos_profile_sensor_data,
        )

        now = time.monotonic()
        self.phase = 'STARTUP'
        self.phase_started_at = now
        self.last_progress_log_at = 0.0
        self.finished = False
        self.exit_code = 2
        self.get_logger().info(
            f'Runtime validation level={self.validation_level}: '
            'STARTUP -> CLICK -> IK\n'
            'Hardware command publishers are forbidden.'
        )

    def _positive_float(self, parameter_name):
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{parameter_name} must be finite and positive')
        return value

    @staticmethod
    def _metadata(message, received_at):
        return StreamMetadata(
            received_at=received_at,
            width=int(message.width),
            height=int(message.height),
            frame_id=str(message.header.frame_id),
            encoding=str(getattr(message, 'encoding', '')),
        )

    def _rgb_callback(self, message):
        now = time.monotonic()
        self.rgb_times.append(now)
        self.rgb = self._metadata(message, now)

    def _depth_callback(self, message):
        now = time.monotonic()
        self.depth_times.append(now)
        self.depth = self._metadata(message, now)

    def _camera_info_callback(self, message):
        self.camera_info = self._metadata(message, time.monotonic())

    def _joint_callback(self, message):
        now = time.monotonic()
        self.joint_times.append(now)
        self.latest_joint_result = validate_joint_solution(
            message.name,
            message.position,
        )

    def _preview_callback(self, message):
        self.preview_message_count += 1
        self.latest_preview_at = time.monotonic()
        result = validate_joint_solution(message.name, message.position)
        self.latest_preview_result = result
        if not result.failed:
            self.preview_valid_count += 1

    def _validate_point(self, message, expected_frame):
        frame = str(message.header.frame_id).strip()
        if frame != expected_frame:
            return CheckResult(
                FAIL,
                f'expected frame {expected_frame!r}, got {frame!r}',
            )
        if _stamp_key(message) == (0, 0):
            return CheckResult(FAIL, 'point has an uninitialized timestamp')
        if not all(math.isfinite(value) for value in _point_xyz(message)):
            return CheckResult(FAIL, 'point contains NaN or infinity')
        return CheckResult(PASS, f'finite point in {frame}')

    @staticmethod
    def _store_recent(mapping, order, key, message, limit=512):
        mapping[key] = message
        order.append(key)
        while len(order) > limit:
            old_key = order.popleft()
            if old_key not in order:
                mapping.pop(old_key, None)

    def _object_callback(self, message):
        result = self._validate_point(message, self.expected_frame)
        if result.failed:
            self.invalid_object_reason = result.detail
            return
        self.valid_object_count += 1
        key = _stamp_key(message)
        self._store_recent(
            self.object_messages,
            self.object_order,
            key,
            message,
        )
        self._try_match_target(key)

    def _base_callback(self, message):
        result = self._validate_point(message, self.base_frame)
        if result.failed:
            self.invalid_base_reason = result.detail
            return
        self.valid_base_count += 1
        key = _stamp_key(message)
        self._store_recent(
            self.base_messages,
            self.base_order,
            key,
            message,
        )
        self._try_match_target(key)

    def _try_match_target(self, key):
        if key in self.matched_stamps:
            return
        camera_point = self.object_messages.get(key)
        base_point = self.base_messages.get(key)
        if camera_point is None or base_point is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.expected_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            expected = transform_xyz(
                _point_xyz(camera_point),
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            error = point_distance(expected, _point_xyz(base_point))
        except (TransformException, ValueError) as exception:
            self.invalid_base_reason = (
                f'cannot verify target transform: {exception}'
            )
            return
        self.matched_stamps.add(key)
        self.transform_errors.append(error)

    def _publisher_names(self, topic):
        return tuple(
            normalize_node_name(info.node_name, info.node_namespace)
            for info in self.get_publishers_info_by_topic(topic)
        )

    def _command_publisher_result(self):
        publishers = {
            topic: self._publisher_names(topic)
            for topic in COMMAND_TOPICS
        }
        return evaluate_command_publishers(
            publishers,
            require_zero=self.require_zero_command_publishers,
        )

    def _node_result(self):
        actual = {
            normalize_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }
        missing = sorted(set(self.required_nodes) - actual)
        if missing:
            return CheckResult(FAIL, f'missing required nodes: {missing}')
        return CheckResult(
            PASS,
            f'all {len(self.required_nodes)} required nodes discovered',
        )

    def _publisher_contract_result(self):
        contracts = (
            (
                self.topics['rgb_topic'],
                'sensor_msgs/msg/Image',
                '/camera/camera',
                False,
            ),
            (
                self.topics['depth_topic'],
                'sensor_msgs/msg/Image',
                '/camera/camera',
                False,
            ),
            (
                self.topics['camera_info_topic'],
                'sensor_msgs/msg/CameraInfo',
                '/camera/camera',
                False,
            ),
            (
                self.topics['object_topic'],
                'geometry_msgs/msg/PointStamped',
                '/click_target_node',
                True,
            ),
            (
                self.topics['base_target_topic'],
                'geometry_msgs/msg/PointStamped',
                '/astra_click_target_transform_node',
                True,
            ),
            (
                self.topics['joint_state_topic'],
                'sensor_msgs/msg/JointState',
                '/arm_driver_node',
                True,
            ),
            (
                self.topics['preview_topic'],
                'sensor_msgs/msg/JointState',
                '/visual_grasp_controller',
                True,
            ),
        )
        errors = []
        for (
            topic,
            expected_type,
            expected_owner,
            require_exactly_one,
        ) in contracts:
            endpoints = self.get_publishers_info_by_topic(topic)
            endpoint_contract = tuple(
                (
                    endpoint.topic_type,
                    normalize_node_name(
                        endpoint.node_name,
                        endpoint.node_namespace,
                    ),
                )
                for endpoint in endpoints
            )
            result = evaluate_topic_publishers(
                topic,
                endpoint_contract,
                expected_type,
                expected_owner,
                require_exactly_one=require_exactly_one,
            )
            if result.failed:
                errors.append(result.detail)
        if errors:
            return CheckResult(FAIL, '; '.join(errors))
        return CheckResult(
            PASS,
            f'{len(contracts)} topic publisher contracts passed',
        )

    def _tf_result(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.expected_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = (
                translation.x,
                translation.y,
                translation.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            if not all(math.isfinite(value) for value in values):
                return CheckResult(FAIL, 'TF contains NaN or infinity')
            norm = math.sqrt(sum(value * value for value in values[3:]))
            if abs(norm - 1.0) > 1.0e-3:
                return CheckResult(
                    FAIL,
                    f'TF quaternion norm is {norm:.6f}',
                )
        except TransformException as exception:
            return CheckResult(
                FAIL,
                f'cannot transform {self.expected_frame} to '
                f'{self.base_frame}: {exception}',
            )
        return CheckResult(
            PASS,
            f'TF {self.base_frame} <- {self.expected_frame} is available',
        )

    def _joint_result(self, now):
        if self.latest_joint_result.failed:
            return self.latest_joint_result
        rate_result = evaluate_rate(
            'joint states',
            self.joint_times,
            now,
            self.rate_window_sec,
            self.joint_state_min_fps,
            1.0,
            self.freshness_sec,
        )
        if rate_result.failed:
            return rate_result
        return CheckResult(
            PASS,
            f'{self.latest_joint_result.detail}; {rate_result.detail}',
        )

    def _evaluate_startup(self, now):
        return {
            'startup.nodes': self._node_result(),
            'startup.publishers': self._publisher_contract_result(),
            'startup.registered_rgbd': evaluate_registered_streams(
                self.rgb,
                self.depth,
                self.camera_info,
                self.expected_width,
                self.expected_height,
                self.expected_frame,
                now,
                self.freshness_sec,
            ),
            'startup.rgb_rate': evaluate_rate(
                'RGB',
                self.rgb_times,
                now,
                self.rate_window_sec,
                self.expected_fps,
                self.minimum_fps_ratio,
                self.freshness_sec,
            ),
            'startup.depth_rate': evaluate_rate(
                'depth',
                self.depth_times,
                now,
                self.rate_window_sec,
                self.expected_fps,
                self.minimum_fps_ratio,
                self.freshness_sec,
            ),
            'startup.joint_states': self._joint_result(now),
            'startup.tf': self._tf_result(),
            'safety.command_publishers': self._command_publisher_result(),
        }

    def _evaluate_click(self, now):
        if self.valid_object_count >= self.required_target_pairs:
            object_result = CheckResult(
                PASS,
                f'{self.valid_object_count} valid camera-frame points',
            )
        else:
            object_result = CheckResult(
                FAIL,
                f'{self.valid_object_count}/{self.required_target_pairs} '
                f'valid camera points; {self.invalid_object_reason}',
            )
        if self.valid_base_count >= self.required_target_pairs:
            base_result = CheckResult(
                PASS,
                f'{self.valid_base_count} valid base-frame points',
            )
        else:
            base_result = CheckResult(
                FAIL,
                f'{self.valid_base_count}/{self.required_target_pairs} '
                f'valid base points; {self.invalid_base_reason}',
            )

        recent_errors = tuple(self.transform_errors)[
            -self.required_target_pairs:
        ]
        if len(recent_errors) < self.required_target_pairs:
            chain_result = CheckResult(
                FAIL,
                f'{len(recent_errors)}/{self.required_target_pairs} '
                'timestamp-matched target pairs',
            )
        else:
            maximum_error = max(recent_errors)
            detail = (
                f'max TF replay error={maximum_error * 1000.0:.3f} mm '
                f'(limit={self.transform_tolerance_m * 1000.0:.3f} mm)'
            )
            chain_result = CheckResult(
                PASS if maximum_error <= self.transform_tolerance_m else FAIL,
                detail,
            )
        return {
            'click.camera_point': object_result,
            'click.base_point': base_result,
            'click.transform_chain': chain_result,
        }

    def _evaluate_ik(self, now):
        if self.preview_valid_count < self.required_preview_solutions:
            return {
                'ik.preview_solutions': CheckResult(
                    FAIL,
                    f'{self.preview_valid_count}/'
                    f'{self.required_preview_solutions} valid previews; '
                    f'{self.latest_preview_result.detail}',
                )
            }
        if self.latest_preview_at is None:
            return {
                'ik.preview_solutions': CheckResult(
                    FAIL,
                    'preview count is inconsistent with receive time',
                )
            }
        age_sec = now - self.latest_preview_at
        if age_sec > self.ik_timeout_sec:
            return {
                'ik.preview_solutions': CheckResult(
                    FAIL,
                    f'latest preview is stale ({age_sec:.3f} s)',
                )
            }
        return {
            'ik.preview_solutions': CheckResult(
                PASS,
                f'{self.preview_valid_count} finite ordered dry-run '
                'solutions received',
            )
        }

    @staticmethod
    def _all_ready(results):
        return all(not result.failed for result in results.values())

    def _log_progress(self, now, message):
        if now - self.last_progress_log_at >= 3.0:
            self.last_progress_log_at = now
            self.get_logger().info(message)

    def step(self):
        """Advance the fail-closed STARTUP, CLICK, and IK state machine."""
        if self.finished:
            return
        now = time.monotonic()
        command_result = self._command_publisher_result()
        if command_result.failed:
            self._finalize(
                CheckResult(FAIL, command_result.detail),
                now,
            )
            return

        elapsed = now - self.phase_started_at
        if self.phase == 'STARTUP':
            startup = self._evaluate_startup(now)
            if self._all_ready(startup):
                if self.validation_level == 'startup':
                    self._finalize(None, now)
                    return
                self.phase = 'CLICK'
                self.phase_started_at = now
                self.get_logger().info(
                    '[PASS] STARTUP complete. Click the intended object '
                    'once in the Astra window.'
                )
                return
            waiting = [
                name for name, result in startup.items() if result.failed
            ]
            self._log_progress(
                now,
                f'STARTUP waiting: {waiting}',
            )
            if elapsed >= self.startup_timeout_sec:
                self._finalize(
                    CheckResult(FAIL, 'STARTUP timeout'),
                    now,
                )
            return

        if self.phase == 'CLICK':
            click = self._evaluate_click(now)
            if self._all_ready(click):
                if self.validation_level == 'click':
                    self._finalize(None, now)
                    return
                self.phase = 'IK'
                self.phase_started_at = now
                self.get_logger().info(
                    '[PASS] CLICK chain complete. Waiting for the full '
                    'dry-run IK sequence.'
                )
                return
            self._log_progress(
                now,
                'CLICK waiting: select one reachable object in the '
                'Astra image window',
            )
            if elapsed >= self.click_timeout_sec:
                self._finalize(
                    CheckResult(FAIL, 'CLICK timeout'),
                    now,
                )
            return

        if self.phase == 'IK':
            ik = self._evaluate_ik(now)
            if self._all_ready(ik):
                self._finalize(None, now)
                return
            self._log_progress(
                now,
                f'IK waiting: {self.preview_valid_count}/'
                f'{self.required_preview_solutions} preview solutions',
            )
            if elapsed >= self.ik_timeout_sec:
                self._finalize(
                    CheckResult(FAIL, 'IK timeout'),
                    now,
                )

    def _finalize(self, phase_failure, now):
        results = {}
        results.update(self._evaluate_startup(now))
        if self.validation_level in ('click', 'ik'):
            results.update(self._evaluate_click(now))
        if self.validation_level == 'ik':
            results.update(self._evaluate_ik(now))
        if phase_failure is not None:
            results[f'runtime.{self.phase.lower()}'] = phase_failure

        pass_count = sum(
            result.status == PASS for result in results.values()
        )
        warn_count = sum(
            result.status == WARN for result in results.values()
        )
        fail_count = sum(
            result.status == FAIL for result in results.values()
        )
        lines = ['=== ASTRA CLICK-GRASP RUNTIME VALIDATION ===']
        lines.extend(
            f'[{result.status}] {name}: {result.detail}'
            for name, result in results.items()
        )
        final_label = PASS if fail_count == 0 else FAIL
        lines.append(
            f'FINAL={final_label} pass={pass_count} '
            f'warn={warn_count} fail={fail_count}'
        )
        summary = '\n'.join(lines)
        if fail_count:
            self.get_logger().error(summary)
            self.exit_code = 1
        else:
            self.get_logger().info(summary)
            self.exit_code = 0
        self.finished = True


def main(args=None):
    """Run one bounded validation and return nonzero on any failed check."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = AstraClickGraspRuntimeValidator()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.05)
            node.step()
        return node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception as exception:
        if node is None:
            print(f'runtime validator could not start: {exception}')
        else:
            node.get_logger().error(
                f'runtime validator aborted: {exception}'
            )
        return 2
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
                executor.shutdown(timeout_sec=1.0)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == '__main__':
    raise SystemExit(main())
