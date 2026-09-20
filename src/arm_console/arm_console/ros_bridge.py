"""Thread-isolated ROS communication for the Arm Console."""

from dataclasses import dataclass
import math
from queue import Empty, SimpleQueue
import threading
import time
from typing import Optional, Sequence

from python_qt_binding.QtCore import QObject, Signal
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, Int32
from std_srvs.srv import SetBool, Trigger

from arm_console.motion_status import MotionStatus
JOINT_NAMES = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
)

LIVE_PUBLISH_PERIOD_MIN_SEC = 0.02
LIVE_PUBLISH_PERIOD_MAX_SEC = 0.5
LIVE_GUI_HEARTBEAT_TIMEOUT_MAX_SEC = 1.0
GRIPPER_MIN_PULSE_US = 600
GRIPPER_MAX_PULSE_US = 1700
J6_INDEX = 5


def validate_timeout_sec(value, parameter_name) -> float:
    """Return a finite positive timeout or fail configuration startup."""
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(
            f'{parameter_name} must be finite and positive'
        )
    return timeout


@dataclass(frozen=True)
class BackendStatus:
    """Current ROS endpoint and joint-state health."""

    aggregate: str
    state_stream: str
    state_age_sec: Optional[float]
    command_ready: bool
    external_command_publishers: int
    stop_ready: bool
    disable_ready: bool
    robot_state_publisher_ready: bool
    rviz_detected: bool
    gripper_ready: bool = False
    preview_ready: bool = False
    recovery_frames: int = 0
    recovery_frames_required: int = 3
    state_publishers: int = 0
    state_source_ready: bool = True
    command_subscribers: int = 1
    live_command_ready: bool = False
    live_command_subscribers: int = 1
    live_service_ready: bool = False
    live_state_ready: bool = False
    live_state_publishers: int = 1
    live_heartbeat_ready: bool = False
    live_heartbeat_subscribers: int = 1
    live_armed: bool = False
    external_live_command_publishers: int = 0
    external_live_heartbeat_publishers: int = 0


class FeedbackTracker:
    """Track fresh feedback and require stable frames before enabling MOVE."""

    def __init__(self, timeout_sec, recovery_frames_required=3):
        self.timeout_sec = validate_timeout_sec(
            timeout_sec,
            'feedback timeout',
        )
        if recovery_frames_required < 1:
            raise ValueError('recovery frame count must be positive')
        self.recovery_frames_required = int(recovery_frames_required)
        self.last_received_at = None
        self._continuous_valid_frames = 0

    def record_valid(self, received_at):
        """Record one valid frame, resetting continuity after a long gap."""
        if (
            self.last_received_at is None
            or received_at < self.last_received_at
            or received_at - self.last_received_at > self.timeout_sec
        ):
            self._continuous_valid_frames = 1
        else:
            self._continuous_valid_frames = min(
                self.recovery_frames_required,
                self._continuous_valid_frames + 1,
            )
        self.last_received_at = received_at

    def record_invalid(self):
        """Break recovery continuity without refreshing the valid-state age."""
        self._continuous_valid_frames = 0

    def snapshot(self, now):
        """Return state age, stream health, and current recovery streak."""
        if self.last_received_at is None:
            return None, 'never', 0

        state_age = max(0.0, now - self.last_received_at)
        if state_age > self.timeout_sec:
            return state_age, 'stale', 0
        if self._continuous_valid_frames < self.recovery_frames_required:
            return state_age, 'recovering', self._continuous_valid_frames
        return state_age, 'fresh', self._continuous_valid_frames


class RosSignals(QObject):
    """Qt signals emitted by the ROS executor thread."""

    joint_state_received = Signal(object)
    backend_status_changed = Signal(object)
    motion_status_changed = Signal(object)
    operation_finished = Signal(str, bool, str)
    log_message = Signal(str, str)


def extract_joint_positions_degrees(
    names: Sequence[str],
    positions: Sequence[float],
) -> tuple[float, ...]:
    """Validate and reorder a JointState position array by joint name."""
    if len(names) != len(positions):
        raise ValueError('JointState name and position lengths differ')
    if len(set(names)) != len(names):
        raise ValueError('JointState contains duplicate joint names')

    position_by_name = dict(zip(names, positions))
    missing = [name for name in JOINT_NAMES if name not in position_by_name]
    if missing:
        raise ValueError('JointState is missing: ' + ', '.join(missing))

    ordered = tuple(float(position_by_name[name]) for name in JOINT_NAMES)
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError('JointState contains nan or infinity')

    return tuple(math.degrees(value) for value in ordered)


def validate_target_degrees(
    target_degrees: Sequence[float],
    joint_min_rad: Sequence[float],
    joint_max_rad: Sequence[float],
) -> tuple[float, ...]:
    """Validate six degree targets and return them in radians."""
    if len(target_degrees) != len(JOINT_NAMES):
        raise ValueError('Exactly six target joint angles are required')
    if len(joint_min_rad) != len(JOINT_NAMES):
        raise ValueError('Exactly six minimum joint limits are required')
    if len(joint_max_rad) != len(JOINT_NAMES):
        raise ValueError('Exactly six maximum joint limits are required')
    if not all(
        math.isfinite(value)
        for value in (*joint_min_rad, *joint_max_rad)
    ):
        raise ValueError('Configured joint limits contain nan or infinity')

    radians = tuple(math.radians(float(value)) for value in target_degrees)
    if not all(math.isfinite(value) for value in radians):
        raise ValueError('Target contains nan or infinity')

    for index, (value, lower, upper) in enumerate(
        zip(radians, joint_min_rad, joint_max_rad)
    ):
        if lower > upper:
            raise ValueError(f'J{index + 1} has invalid configured limits')
        if index != J6_INDEX and (value < lower or value > upper):
            lower_deg = math.degrees(lower)
            upper_deg = math.degrees(upper)
            raise ValueError(
                f'J{index + 1} must be between '
                f'{lower_deg:.1f} and {upper_deg:.1f} deg'
            )

    return radians


def validate_gripper_pulse(pulse_us, minimum, maximum) -> int:
    """Return an integral gripper pulse inside the configured range."""
    value = float(pulse_us)
    if not math.isfinite(value) or not value.is_integer():
        raise ValueError('Gripper pulse must be a finite integer')
    pulse = int(value)
    if minimum > maximum:
        raise ValueError('Configured gripper pulse range is invalid')
    if pulse < minimum or pulse > maximum:
        raise ValueError(
            f'Gripper pulse must be between {minimum} and {maximum} μs'
        )
    return pulse


def validate_live_timing(
    publish_period_sec,
    gui_heartbeat_timeout_sec,
) -> tuple[float, float]:
    """Validate and return bounded LIVE FOLLOW timing parameters."""
    publish_period = float(publish_period_sec)
    heartbeat_timeout = float(gui_heartbeat_timeout_sec)
    if not math.isfinite(publish_period):
        raise ValueError('live_publish_period_sec must be finite')
    if not (
        LIVE_PUBLISH_PERIOD_MIN_SEC
        <= publish_period
        <= LIVE_PUBLISH_PERIOD_MAX_SEC
    ):
        raise ValueError(
            'live_publish_period_sec must be between '
            f'{LIVE_PUBLISH_PERIOD_MIN_SEC} and '
            f'{LIVE_PUBLISH_PERIOD_MAX_SEC} seconds'
        )
    if not math.isfinite(heartbeat_timeout):
        raise ValueError(
            'live_gui_heartbeat_timeout_sec must be finite'
        )
    if heartbeat_timeout < publish_period * 2.0:
        raise ValueError(
            'live_gui_heartbeat_timeout_sec must allow two publish periods'
        )
    if heartbeat_timeout > LIVE_GUI_HEARTBEAT_TIMEOUT_MAX_SEC:
        raise ValueError(
            'live_gui_heartbeat_timeout_sec must be no greater than '
            f'{LIVE_GUI_HEARTBEAT_TIMEOUT_MAX_SEC} second'
        )
    return publish_period, heartbeat_timeout


def classify_backend(
    state_age_sec: Optional[float],
    state_timeout_sec: float,
    command_ready: bool,
    stop_ready: bool,
    disable_ready: bool,
    command_conflict: bool = False,
    recovery_frames: Optional[int] = None,
    recovery_frames_required: int = 3,
) -> tuple[str, str]:
    """Return operator-facing ARM health and state-stream labels."""
    if state_age_sec is None:
        return 'waiting', 'never'
    if state_age_sec > state_timeout_sec:
        return 'feedback_lost', 'stale'
    if (
        recovery_frames is not None
        and recovery_frames < recovery_frames_required
    ):
        return 'recovering', 'recovering'
    if (
        command_ready
        and stop_ready
        and disable_ready
        and not command_conflict
    ):
        return 'ready', 'fresh'
    return 'degraded', 'fresh'


def _arm_degraded_detail(status):
    missing = []
    if not status.state_source_ready:
        missing.append(
            f'{status.state_publishers} joint-state publisher(s)'
        )
    if status.command_subscribers > 1:
        missing.append(
            f'{status.command_subscribers} command subscribers'
        )
    elif not status.command_ready:
        missing.append('command subscriber')
    if status.external_command_publishers > 0:
        missing.append(
            f'{status.external_command_publishers} '
            'external command publisher(s)'
        )
    if not status.stop_ready:
        missing.append('STOP service')
    if not status.disable_ready:
        missing.append('DISABLE service')
    return ', '.join(missing) or 'control route incomplete'


def backend_transition_events(previous, current):
    """Describe ARM and gripper component transitions independently."""
    if previous is None:
        return ()

    events = []
    if current.aggregate != previous.aggregate:
        if current.aggregate == 'ready':
            events.append((
                'info',
                'ARM READY: feedback and safety route stable',
            ))
        elif current.aggregate == 'recovering':
            events.append((
                'warning',
                'ARM RECOVERING: waiting for consecutive valid feedback; '
                'MOVE remains locked',
            ))
        elif current.aggregate == 'feedback_lost':
            route_detail = (
                '/joint_states has no publisher'
                if current.state_publishers == 0
                else '/joint_states stopped updating'
            )
            events.append((
                'error',
                f'ARM FEEDBACK LOST: {route_detail}; MOVE locked',
            ))
        elif current.aggregate == 'waiting':
            events.append((
                'warning',
                'ARM WAITING: no valid /joint_states frame received',
            ))
        elif current.aggregate == 'degraded':
            events.append((
                'warning',
                'ARM DEGRADED: ' + _arm_degraded_detail(current),
            ))
    elif (
        current.aggregate == 'degraded'
        and (
            current.command_ready,
            current.command_subscribers,
            current.external_command_publishers,
            current.state_publishers,
            current.stop_ready,
            current.disable_ready,
        )
        != (
            previous.command_ready,
            previous.command_subscribers,
            previous.external_command_publishers,
            previous.state_publishers,
            previous.stop_ready,
            previous.disable_ready,
        )
    ):
        events.append((
            'warning',
            'ARM DEGRADED: ' + _arm_degraded_detail(current),
        ))

    if current.gripper_ready != previous.gripper_ready:
        events.append((
            'info' if current.gripper_ready else 'warning',
            'GRIPPER ROUTE: command subscriber detected'
            if current.gripper_ready
            else 'GRIPPER ROUTE: no command subscriber',
        ))
    if current.live_armed != previous.live_armed:
        events.append((
            'warning' if current.live_armed else 'info',
            'LIVE FOLLOW: ARMED; physical sliders control the arm'
            if current.live_armed
            else 'LIVE FOLLOW: DISARMED',
        ))
    return tuple(events)


def move_block_reason(status):
    """Return a safety reason when the ARM route cannot accept MOVE."""
    if status.state_stream == 'never':
        return 'MOVE blocked: waiting for valid ARM feedback'
    if status.state_stream == 'stale':
        return 'MOVE blocked: ARM feedback is lost'
    if status.state_stream == 'recovering':
        return (
            'MOVE blocked: ARM feedback recovery requires '
            f'{status.recovery_frames_required} consecutive valid frames '
            f'({status.recovery_frames}/{status.recovery_frames_required})'
        )
    if not status.state_source_ready:
        if status.state_publishers > 1:
            return 'MOVE blocked: multiple /joint_states publishers detected'
        return 'MOVE blocked: /joint_states publisher is not detected'
    if status.live_armed:
        return 'MOVE blocked: LIVE FOLLOW is armed; return to PREVIEW first'
    if status.command_subscribers > 1:
        return 'MOVE blocked: multiple /arm/joint_command subscribers detected'
    if not status.command_ready:
        return 'MOVE blocked: /arm/joint_command has no subscriber'
    if status.external_command_publishers > 0:
        return 'MOVE blocked: another command publisher is active'
    if not status.stop_ready or not status.disable_ready:
        return 'MOVE blocked: ARM safety services are not discovered'
    return None


def live_follow_block_reason(status):
    """Return the reason a new LIVE FOLLOW session cannot be armed."""
    base_reason = move_block_reason(status)
    if base_reason is not None and not status.live_armed:
        return base_reason.replace('MOVE blocked:', 'LIVE FOLLOW blocked:', 1)
    if status.live_armed:
        return None
    if status.live_command_subscribers > 1:
        return 'LIVE FOLLOW blocked: multiple live command subscribers'
    if not status.live_command_ready:
        return (
            'LIVE FOLLOW blocked: /arm/live_joint_command has no subscriber'
        )
    if not status.live_service_ready:
        return 'LIVE FOLLOW blocked: /arm/live_follow service not discovered'
    if not status.live_state_ready:
        return 'LIVE FOLLOW blocked: driver state publisher not detected'
    if status.live_state_publishers > 1:
        return 'LIVE FOLLOW blocked: multiple driver state publishers'
    if not status.live_heartbeat_ready:
        return 'LIVE FOLLOW blocked: driver heartbeat route not detected'
    if status.live_heartbeat_subscribers > 1:
        return 'LIVE FOLLOW blocked: multiple heartbeat subscribers'
    if status.external_live_command_publishers > 0:
        return 'LIVE FOLLOW blocked: another live command publisher is active'
    if status.external_live_heartbeat_publishers > 0:
        return 'LIVE FOLLOW blocked: another heartbeat publisher is active'
    return None


def live_runtime_block_reason(status):
    """Return a fail-closed reason for an already armed LIVE session."""
    if status.state_stream == 'never':
        return 'ARM feedback has never produced a valid frame'
    if status.state_stream == 'stale':
        return 'ARM feedback is stale'
    if status.state_stream == 'recovering':
        return 'ARM feedback is recovering'
    if status.state_stream != 'fresh':
        return f'ARM feedback state is {status.state_stream}'
    if status.state_publishers != 1 or not status.state_source_ready:
        return (
            'ARM feedback source conflict: expected exactly one '
            f'/joint_states publisher, found {status.state_publishers}'
        )
    if status.command_subscribers != 1 or not status.command_ready:
        return (
            'ARM control route conflict: expected exactly one '
            f'/arm/joint_command subscriber, found '
            f'{status.command_subscribers}'
        )
    if status.external_command_publishers > 0:
        return 'ARM control route conflict: external MOVE publisher detected'
    if not status.stop_ready or not status.disable_ready:
        return 'ARM safety service route is incomplete'
    if (
        status.live_command_subscribers != 1
        or not status.live_command_ready
    ):
        return (
            'LIVE command route conflict: expected exactly one subscriber, '
            f'found {status.live_command_subscribers}'
        )
    if not status.live_service_ready:
        return 'LIVE FOLLOW control service is not discovered'
    if status.live_state_publishers != 1 or not status.live_state_ready:
        return (
            'LIVE state source conflict: expected exactly one publisher, '
            f'found {status.live_state_publishers}'
        )
    if (
        status.live_heartbeat_subscribers != 1
        or not status.live_heartbeat_ready
    ):
        return (
            'LIVE heartbeat route conflict: expected exactly one subscriber, '
            f'found {status.live_heartbeat_subscribers}'
        )
    if status.external_live_command_publishers > 0:
        return 'LIVE command route conflict: external publisher detected'
    if status.external_live_heartbeat_publishers > 0:
        return 'LIVE heartbeat route conflict: external publisher detected'
    return None


class ArmConsoleNode(Node):
    """ROS node used only through the thread-safe request methods below."""

    def __init__(self, signals: RosSignals):
        super().__init__('arm_console')
        self.signals = signals

        self.declare_parameter('state_timeout_sec', 1.0)
        self.declare_parameter('recovery_valid_frames', 3)
        self.declare_parameter('service_timeout_sec', 2.0)
        self.declare_parameter('live_publish_period_sec', 0.1)
        self.declare_parameter('live_gui_heartbeat_timeout_sec', 0.25)
        self.declare_parameter('joint_min_rad', [-3.14] * 6)
        self.declare_parameter('joint_max_rad', [3.14] * 6)
        self.declare_parameter('gripper_open_pulse_us', 600)
        self.declare_parameter('gripper_close_pulse_us', 1300)

        self.state_timeout_sec = validate_timeout_sec(
            self.get_parameter('state_timeout_sec').value,
            'state_timeout_sec',
        )
        self.service_timeout_sec = validate_timeout_sec(
            self.get_parameter('service_timeout_sec').value,
            'service_timeout_sec',
        )
        self.live_publish_period_sec = float(
            self.get_parameter('live_publish_period_sec').value
        )
        self.live_gui_heartbeat_timeout_sec = float(
            self.get_parameter('live_gui_heartbeat_timeout_sec').value
        )
        (
            self.live_publish_period_sec,
            self.live_gui_heartbeat_timeout_sec,
        ) = validate_live_timing(
            self.live_publish_period_sec,
            self.live_gui_heartbeat_timeout_sec,
        )
        self.recovery_valid_frames = int(
            self.get_parameter('recovery_valid_frames').value
        )
        self.joint_min_rad = tuple(
            float(value)
            for value in self.get_parameter('joint_min_rad').value
        )
        self.joint_max_rad = tuple(
            float(value)
            for value in self.get_parameter('joint_max_rad').value
        )
        self.gripper_open_pulse_us = int(
            self.get_parameter('gripper_open_pulse_us').value
        )
        self.gripper_close_pulse_us = int(
            self.get_parameter('gripper_close_pulse_us').value
        )
        self.gripper_min_pulse_us = GRIPPER_MIN_PULSE_US
        self.gripper_max_pulse_us = GRIPPER_MAX_PULSE_US
        validate_gripper_pulse(
            self.gripper_open_pulse_us,
            self.gripper_min_pulse_us,
            self.gripper_max_pulse_us,
        )
        validate_gripper_pulse(
            self.gripper_close_pulse_us,
            self.gripper_min_pulse_us,
            self.gripper_max_pulse_us,
        )
        configured_midpoints_deg = [
            math.degrees((lower + upper) / 2.0)
            for lower, upper in zip(
                self.joint_min_rad,
                self.joint_max_rad,
            )
        ]
        validate_target_degrees(
            configured_midpoints_deg,
            self.joint_min_rad,
            self.joint_max_rad,
        )

        if self.recovery_valid_frames < 1:
            raise ValueError('recovery_valid_frames must be positive')

        self.command_publisher = self.create_publisher(
            JointState,
            '/arm/joint_command',
            10,
        )
        self.preview_publisher = self.create_publisher(
            JointState,
            '/arm/preview_joint_states',
            10,
        )
        live_command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        live_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.live_command_publisher = self.create_publisher(
            JointState,
            '/arm/live_joint_command',
            live_command_qos,
        )
        self.live_heartbeat_publisher = self.create_publisher(
            Header,
            '/arm/live_follow_heartbeat',
            live_command_qos,
        )
        self.gripper_publisher = self.create_publisher(
            Int32,
            '/gripper/pulse_us',
            10,
        )
        self.state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )
        self.live_state_subscription = self.create_subscription(
            Bool,
            '/arm/live_follow_state',
            self._live_state_callback,
            live_state_qos,
        )
        self.stop_client = self.create_client(Trigger, '/arm/stop')
        self.disable_client = self.create_client(Trigger, '/arm/disable')
        self.live_client = self.create_client(SetBool, '/arm/live_follow')

        self._request_queue = SimpleQueue()
        self._pending_services = {}
        self._safety_action_lock = threading.Lock()
        self._safety_actions_pending = set()
        self._feedback_tracker = FeedbackTracker(
            self.state_timeout_sec,
            self.recovery_valid_frames,
        )
        self._last_invalid_state_log_at = 0.0
        self._latest_joint_radians = None
        self._latest_joint_received_at = None
        self._last_status = None
        self._preview_target_radians = None
        self._live_target_lock = threading.Lock()
        self._live_target = None
        self._live_gui_heartbeat_at = None
        self._live_armed = False
        self._live_state_seen = False
        self._live_safety_trip_latched = False
        self._live_safety_disarm_requested = False
        self._live_disarmed_event = threading.Event()
        self._live_disarmed_event.set()
        self.request_timer = self.create_timer(0.05, self._drain_requests)
        self.preview_timer = self.create_timer(0.05, self._publish_preview)
        self.live_timer = self.create_timer(
            self.live_publish_period_sec,
            self._publish_live_target,
        )
        self.status_timer = self.create_timer(0.20, self._publish_status)

    @property
    def joint_limits_degrees(self):
        """Return the configured six-axis limits for UI construction."""
        return tuple(
            (math.degrees(lower), math.degrees(upper))
            for lower, upper in zip(self.joint_min_rad, self.joint_max_rad)
        )

    def request_move(self, target_degrees):
        """Queue a MOVE request without touching ROS from the Qt thread."""
        self._request_queue.put(('move', tuple(target_degrees)))

    def request_stop(self):
        """Queue a STOP service request."""
        self._begin_safety_action('stop')
        self._request_queue.put(('stop', None))

    def request_disable(self):
        """Queue a DISABLE service request."""
        self._begin_safety_action('disable')
        self._request_queue.put(('disable', None))

    def request_preview(self, target_degrees):
        """Queue a visualization-only target update."""
        self._request_queue.put(('preview', tuple(target_degrees)))

    def request_gripper(self, pulse_us):
        """Queue one explicit gripper pulse command."""
        self._request_queue.put(('gripper', pulse_us))

    def request_live_mode(self, enabled, baseline_degrees=None):
        """Queue an explicit LIVE FOLLOW arm/disarm request."""
        if enabled:
            with self._safety_action_lock:
                if (
                    not self._safety_actions_pending
                    and not self._live_armed
                ):
                    with self._live_target_lock:
                        self._live_safety_trip_latched = False
                        self._live_safety_disarm_requested = False
                        self._live_target = None
                        self._live_gui_heartbeat_at = None
        else:
            self._inhibit_live_output()
        baseline = (
            None if baseline_degrees is None
            else tuple(baseline_degrees)
        )
        self._request_queue.put(('live_mode', (bool(enabled), baseline)))

    def request_live_target(self, target_degrees):
        """Replace the complete six-axis live target without accumulating."""
        with self._live_target_lock:
            if not self._live_armed or self._live_safety_trip_latched:
                return
            self._live_target = tuple(target_degrees)
            self._live_gui_heartbeat_at = time.monotonic()

    def request_live_heartbeat(self):
        """Record liveness of the Qt event loop without queuing a command."""
        with self._live_target_lock:
            if not self._live_armed or self._live_safety_trip_latched:
                return
            self._live_gui_heartbeat_at = time.monotonic()

    def _clear_live_output(self):
        """Clear the latest live target and GUI deadman timestamp."""
        with self._live_target_lock:
            self._live_target = None
            self._live_gui_heartbeat_at = None

    def _inhibit_live_output(self):
        """Immediately latch off LIVE output for an operator safety action."""
        with self._live_target_lock:
            self._live_safety_trip_latched = True
            self._live_target = None
            self._live_gui_heartbeat_at = None

    def _begin_safety_action(self, operation):
        """Atomically block new motion before queuing STOP or DISABLE."""
        with self._safety_action_lock:
            self._safety_actions_pending.add(operation)
            with self._live_target_lock:
                self._live_safety_trip_latched = True
                self._live_target = None
                self._live_gui_heartbeat_at = None

    def _finish_safety_action(self, operation):
        """Release only the completed STOP or DISABLE pending marker."""
        with self._safety_action_lock:
            self._safety_actions_pending.discard(operation)

    def _safety_action_pending(self):
        """Return whether an operator safety service is unresolved."""
        with self._safety_action_lock:
            return bool(self._safety_actions_pending)

    def _mark_live_disarmed(self):
        """Reset local LIVE state after the driver confirms disarm."""
        self._live_armed = False
        self._live_disarmed_event.set()
        with self._live_target_lock:
            self._live_target = None
            self._live_gui_heartbeat_at = None
            self._live_safety_trip_latched = False
            self._live_safety_disarm_requested = False

    def _request_latched_live_disarm(self):
        """Send at most one disarm request for the current safety trip."""
        if not self._live_armed:
            return
        if not self._live_safety_trip_latched:
            return
        if self._live_safety_disarm_requested:
            return
        if 'live_enable' in self._pending_services:
            return
        if 'live_disable' in self._pending_services:
            return
        self._live_safety_disarm_requested = True
        self._call_live_follow(False, None)

    def _trip_live_follow(self, reason):
        """Fail closed, cease heartbeats, and request driver disarm once."""
        with self._live_target_lock:
            first_trip = not self._live_safety_trip_latched
            self._live_safety_trip_latched = True
            self._live_target = None
            self._live_gui_heartbeat_at = None
        if first_trip:
            self.signals.log_message.emit(
                'error',
                f'LIVE FOLLOW safety trip: {reason}; heartbeat stopped',
            )
        self._request_latched_live_disarm()

    def _joint_state_callback(self, message):
        try:
            degrees = extract_joint_positions_degrees(
                message.name,
                message.position,
            )
        except ValueError as error:
            self._feedback_tracker.record_invalid()
            now = time.monotonic()
            if now - self._last_invalid_state_log_at >= 5.0:
                self._last_invalid_state_log_at = now
                self.signals.log_message.emit(
                    'warning',
                    f'Ignored invalid /joint_states: {error}',
                )
            return

        now = time.monotonic()
        self._feedback_tracker.record_valid(now)
        self._latest_joint_radians = tuple(
            math.radians(value) for value in degrees
        )
        self._latest_joint_received_at = now
        self.signals.joint_state_received.emit(degrees)

    def _live_state_callback(self, message):
        armed = bool(message.data)
        changed = armed != self._live_armed or not self._live_state_seen
        self._live_state_seen = True
        if armed:
            self._live_armed = True
            self._live_disarmed_event.clear()
            self._request_latched_live_disarm()
        else:
            self._mark_live_disarmed()
        if changed:
            level = 'warning' if armed else 'info'
            detail = 'ARMED' if armed else 'DISARMED'
            self.signals.log_message.emit(
                level,
                f'LIVE FOLLOW driver state: {detail}',
            )

    def _drain_requests(self):
        while True:
            try:
                operation, payload = self._request_queue.get_nowait()
            except Empty:
                return

            if operation == 'move':
                self._publish_move(payload)
            elif operation == 'stop':
                self._call_trigger('stop', self.stop_client)
            elif operation == 'disable':
                self._call_trigger('disable', self.disable_client)
            elif operation == 'preview':
                self._update_preview(payload)
            elif operation == 'gripper':
                self._publish_gripper(payload)
            elif operation == 'live_mode':
                enabled, baseline = payload
                self._call_live_follow(enabled, baseline)

    def _update_preview(self, target_degrees):
        try:
            self._preview_target_radians = validate_target_degrees(
                target_degrees,
                self.joint_min_rad,
                self.joint_max_rad,
            )
        except (TypeError, ValueError) as error:
            self.signals.log_message.emit(
                'warning',
                f'Target preview ignored: {error}',
            )

    def _publish_preview(self):
        if self._preview_target_radians is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(self._preview_target_radians)
        self.preview_publisher.publish(message)

    def _publish_live_target(self):
        if not self._live_armed:
            return

        status = self._build_status()
        blocked_reason = live_runtime_block_reason(status)
        if blocked_reason is not None:
            self._trip_live_follow(blocked_reason)
            return

        now = time.monotonic()
        with self._live_target_lock:
            if self._live_safety_trip_latched:
                return
            live_target = self._live_target
            heartbeat_at = self._live_gui_heartbeat_at
            if heartbeat_at is None:
                return
            if now - heartbeat_at > self.live_gui_heartbeat_timeout_sec:
                return

            heartbeat = Header()
            heartbeat.stamp = self.get_clock().now().to_msg()
            heartbeat.frame_id = 'arm_console_gui_deadman'
            try:
                self.live_heartbeat_publisher.publish(heartbeat)
            except Exception as error:
                self.signals.log_message.emit(
                    'error',
                    f'LIVE FOLLOW heartbeat publish failed: {error}',
                )
                return

            if live_target is None:
                return

            try:
                target_radians = validate_target_degrees(
                    live_target,
                    self.joint_min_rad,
                    self.joint_max_rad,
                )
            except (TypeError, ValueError) as error:
                self.signals.log_message.emit(
                    'warning',
                    f'LIVE FOLLOW target ignored: {error}',
                )
                return

            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = list(target_radians)
            try:
                self.live_command_publisher.publish(message)
            except Exception as error:
                self.signals.log_message.emit(
                    'error',
                    f'LIVE FOLLOW publish failed: {error}',
                )

    def _call_live_follow(self, enabled, baseline_degrees):
        operation = 'live_enable' if enabled else 'live_disable'
        if enabled and 'live_disable' in self._pending_services:
            self.signals.operation_finished.emit(
                operation,
                False,
                'LIVE FOLLOW blocked: disarm is pending',
            )
            return
        if 'live_enable' in self._pending_services or (
            'live_disable' in self._pending_services
        ):
            self.signals.operation_finished.emit(
                operation,
                False,
                'A LIVE FOLLOW mode change is already pending',
            )
            return

        if enabled:
            if self._safety_action_pending():
                self.signals.operation_finished.emit(
                    operation,
                    False,
                    'LIVE FOLLOW blocked: STOP or DISABLE is pending',
                )
                return
            if self._live_armed:
                self.signals.operation_finished.emit(
                    operation,
                    False,
                    'LIVE FOLLOW blocked: driver still reports armed; '
                    'disarm first',
                )
                return
            with self._live_target_lock:
                operator_cancelled = self._live_safety_trip_latched
            if operator_cancelled:
                self.signals.operation_finished.emit(
                    operation,
                    False,
                    'LIVE FOLLOW arming cancelled by a safety action',
                )
                return
            status = self._build_status()
            blocked_reason = live_follow_block_reason(status)
            if blocked_reason is not None:
                self.signals.operation_finished.emit(
                    operation,
                    False,
                    blocked_reason,
                )
                return
            if baseline_degrees is None:
                self.signals.operation_finished.emit(
                    operation,
                    False,
                    'LIVE FOLLOW blocked: no current joint baseline',
                )
                return
            try:
                validate_target_degrees(
                    baseline_degrees,
                    self.joint_min_rad,
                    self.joint_max_rad,
                )
            except (TypeError, ValueError) as error:
                self.signals.operation_finished.emit(
                    operation,
                    False,
                    f'LIVE FOLLOW blocked: {error}',
                )
                return
            self._clear_live_output()
        else:
            self._inhibit_live_output()

        if not self.live_client.service_is_ready():
            if not enabled:
                self._inhibit_live_output()
            self.signals.operation_finished.emit(
                operation,
                False,
                '/arm/live_follow service not discovered',
            )
            return

        request = SetBool.Request()
        request.data = enabled
        safety_blocked = False
        if enabled:
            with self._safety_action_lock:
                if self._safety_actions_pending:
                    safety_blocked = True
                    future = None
                else:
                    future = self.live_client.call_async(request)
        else:
            future = self.live_client.call_async(request)
        if safety_blocked:
            self.signals.operation_finished.emit(
                operation,
                False,
                'LIVE FOLLOW blocked: STOP or DISABLE is pending',
            )
            return
        deadline = time.monotonic() + self.service_timeout_sec
        self._pending_services[operation] = (future, deadline)
        future.add_done_callback(
            lambda completed, name=operation, requested=enabled:
            self._live_service_completed(name, requested, completed)
        )

    def _live_service_completed(self, operation, enabled, future):
        pending = self._pending_services.get(operation)
        if pending is None or pending[0] is not future:
            return
        self._pending_services.pop(operation, None)

        try:
            response = future.result()
            success = bool(response.success)
            detail = response.message or 'No response detail'
        except Exception as error:
            success = False
            detail = str(error)

        if success:
            self._live_state_seen = True
            if enabled:
                self._live_armed = True
                self._live_disarmed_event.clear()
                motion = MotionStatus(
                    state='live',
                    detail='LIVE FOLLOW armed by the backend',
                )
            else:
                self._mark_live_disarmed()
                motion = MotionStatus(
                    state='stopped',
                    detail='LIVE FOLLOW disarmed and STOP confirmed',
                )
            self.signals.motion_status_changed.emit(motion)
        elif enabled:
            self._clear_live_output()

        action = 'armed' if enabled else 'disarmed'
        result = 'succeeded' if success else 'failed'
        self.signals.operation_finished.emit(
            operation,
            success,
            f'LIVE FOLLOW {action} {result}: {detail}',
        )
        if enabled and self._live_safety_trip_latched:
            self._request_latched_live_disarm()

    def _publish_gripper(self, pulse_us):
        if self.count_subscribers('/gripper/pulse_us') == 0:
            self.signals.operation_finished.emit(
                'gripper',
                False,
                'Gripper command blocked: /gripper/pulse_us has no subscriber',
            )
            return
        try:
            pulse = validate_gripper_pulse(
                pulse_us,
                self.gripper_min_pulse_us,
                self.gripper_max_pulse_us,
            )
        except (TypeError, ValueError) as error:
            self.signals.operation_finished.emit(
                'gripper',
                False,
                f'Gripper command blocked: {error}',
            )
            return

        message = Int32()
        message.data = pulse
        try:
            self.gripper_publisher.publish(message)
        except Exception as error:
            self.signals.operation_finished.emit(
                'gripper',
                False,
                f'Gripper publish failed: {error}',
            )
            return

        self.signals.operation_finished.emit(
            'gripper',
            True,
            f'Gripper command published: {pulse} μs (no position feedback)',
        )

    def _publish_move(self, target_degrees):
        if 'live_disable' in self._pending_services:
            self.signals.operation_finished.emit(
                'move',
                False,
                'MOVE blocked: LIVE FOLLOW disarm is pending',
            )
            return
        if self._safety_action_pending():
            self.signals.operation_finished.emit(
                'move',
                False,
                'MOVE blocked: STOP or DISABLE is pending',
            )
            return
        status = self._build_status()
        blocked_reason = move_block_reason(status)
        if blocked_reason is not None:
            self.signals.operation_finished.emit(
                'move',
                False,
                blocked_reason,
            )
            return

        try:
            target_radians = validate_target_degrees(
                target_degrees,
                self.joint_min_rad,
                self.joint_max_rad,
            )
        except (TypeError, ValueError) as error:
            self.signals.operation_finished.emit(
                'move',
                False,
                f'MOVE blocked: {error}',
            )
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(target_radians)
        safety_blocked = False
        publish_error = None
        with self._safety_action_lock:
            if (
                self._safety_actions_pending
                or 'live_disable' in self._pending_services
            ):
                safety_blocked = True
            else:
                try:
                    self.command_publisher.publish(message)
                except Exception as error:
                    publish_error = error
        if safety_blocked:
            if 'live_disable' in self._pending_services:
                blocked_message = (
                    'MOVE blocked: LIVE FOLLOW disarm is pending'
                )
            else:
                blocked_message = 'MOVE blocked: STOP or DISABLE is pending'
            self.signals.operation_finished.emit(
                'move',
                False,
                blocked_message,
            )
            return
        if publish_error is not None:
            self.signals.operation_finished.emit(
                'move',
                False,
                f'MOVE publish failed: {publish_error}',
            )
            return

        self.signals.motion_status_changed.emit(
            MotionStatus(
                state='commanded',
                detail='Command published to /arm/joint_command',
            )
        )

        formatted = '  '.join(
            f'J{index + 1}={value:.1f}°'
            for index, value in enumerate(target_degrees)
        )
        self.signals.operation_finished.emit(
            'move',
            True,
            'Command published to ROS: ' + formatted,
        )

    def _call_trigger(self, operation, client):
        if operation in ('stop', 'disable'):
            self._inhibit_live_output()
        if operation in self._pending_services:
            self.signals.operation_finished.emit(
                operation,
                False,
                f'{operation.upper()} request is already pending',
            )
            return
        if not client.service_is_ready():
            self._finish_safety_action(operation)
            self.signals.operation_finished.emit(
                operation,
                False,
                f'/arm/{operation} service not discovered',
            )
            return

        try:
            future = client.call_async(Trigger.Request())
        except Exception as error:
            self._finish_safety_action(operation)
            self.signals.operation_finished.emit(
                operation,
                False,
                f'/arm/{operation} request failed: {error}',
            )
            return
        deadline = time.monotonic() + self.service_timeout_sec
        self._pending_services[operation] = (future, deadline)
        future.add_done_callback(
            lambda completed, name=operation: self._service_completed(
                name,
                completed,
            )
        )

    def _service_completed(self, operation, future):
        pending = self._pending_services.get(operation)
        if pending is None or pending[0] is not future:
            return
        self._pending_services.pop(operation, None)
        self._finish_safety_action(operation)

        try:
            response = future.result()
            success = bool(response.success)
            detail = response.message or 'No response detail'
        except Exception as error:
            # rclpy futures can surface transport errors.
            success = False
            detail = str(error)

        verb = 'STOP' if operation == 'stop' else 'DISABLE'
        result = 'succeeded' if success else 'failed'
        if success:
            self._mark_live_disarmed()
            state = 'stopped' if operation == 'stop' else 'disabled'
            self.signals.motion_status_changed.emit(
                MotionStatus(
                    state=state,
                    detail=f'{verb} confirmed by the backend',
                )
            )
        self.signals.operation_finished.emit(
            operation,
            success,
            f'{verb} {result}: {detail}',
        )

    def _build_status(self):
        now = time.monotonic()
        state_age, _stream, recovery_frames = (
            self._feedback_tracker.snapshot(now)
        )

        command_subscribers = self.count_subscribers('/arm/joint_command')
        command_ready = command_subscribers == 1
        state_publishers = self.count_publishers('/joint_states')
        state_source_ready = state_publishers == 1
        command_publishers = self.count_publishers('/arm/joint_command')
        external_command_publishers = max(0, command_publishers - 1)
        stop_ready = self.stop_client.service_is_ready()
        disable_ready = self.disable_client.service_is_ready()
        gripper_ready = self.count_subscribers('/gripper/pulse_us') > 0
        preview_ready = (
            self.count_subscribers('/arm/preview_joint_states') > 0
        )
        live_command_subscribers = self.count_subscribers(
            '/arm/live_joint_command'
        )
        live_command_ready = live_command_subscribers == 1
        live_command_publishers = self.count_publishers(
            '/arm/live_joint_command'
        )
        external_live_command_publishers = max(
            0,
            live_command_publishers - 1,
        )
        live_state_publishers = self.count_publishers(
            '/arm/live_follow_state'
        )
        live_state_ready = live_state_publishers == 1
        live_heartbeat_subscribers = self.count_subscribers(
            '/arm/live_follow_heartbeat'
        )
        live_heartbeat_ready = live_heartbeat_subscribers == 1
        live_heartbeat_publishers = self.count_publishers(
            '/arm/live_follow_heartbeat'
        )
        external_live_heartbeat_publishers = max(
            0,
            live_heartbeat_publishers - 1,
        )
        aggregate, state_stream = classify_backend(
            state_age,
            self.state_timeout_sec,
            command_ready,
            stop_ready,
            disable_ready,
            external_command_publishers > 0,
            recovery_frames,
            self.recovery_valid_frames,
        )
        if aggregate == 'ready' and not state_source_ready:
            aggregate = 'degraded'

        try:
            node_names = {
                name for name, _namespace
                in self.get_node_names_and_namespaces()
            }
        except Exception:
            node_names = set()

        return BackendStatus(
            aggregate=aggregate,
            state_stream=state_stream,
            state_age_sec=state_age,
            command_ready=command_ready,
            external_command_publishers=external_command_publishers,
            stop_ready=stop_ready,
            disable_ready=disable_ready,
            robot_state_publisher_ready=(
                'robot_state_publisher' in node_names
            ),
            rviz_detected=any(
                name == 'rviz2' or name.startswith('rviz2_')
                for name in node_names
            ),
            gripper_ready=gripper_ready,
            preview_ready=preview_ready,
            recovery_frames=recovery_frames,
            recovery_frames_required=self.recovery_valid_frames,
            state_publishers=state_publishers,
            state_source_ready=state_source_ready,
            command_subscribers=command_subscribers,
            live_command_ready=live_command_ready,
            live_command_subscribers=live_command_subscribers,
            live_service_ready=self.live_client.service_is_ready(),
            live_state_ready=live_state_ready,
            live_state_publishers=live_state_publishers,
            live_heartbeat_ready=live_heartbeat_ready,
            live_heartbeat_subscribers=live_heartbeat_subscribers,
            live_armed=self._live_armed,
            external_live_command_publishers=(
                external_live_command_publishers
            ),
            external_live_heartbeat_publishers=(
                external_live_heartbeat_publishers
            ),
        )

    def _publish_status(self):
        now = time.monotonic()
        for operation, (future, deadline) in list(
            self._pending_services.items()
        ):
            if now <= deadline or future.done():
                continue
            self._pending_services.pop(operation, None)
            future.cancel()
            if operation in ('stop', 'disable'):
                self._finish_safety_action(operation)
            if operation == 'live_enable':
                with self._live_target_lock:
                    self._live_target = None
                    self._live_gui_heartbeat_at = None
            self.signals.operation_finished.emit(
                operation,
                False,
                f'{operation.upper()} timed out',
            )

        status = self._build_status()
        if status.live_armed:
            blocked_reason = live_runtime_block_reason(status)
            if blocked_reason is not None:
                self._trip_live_follow(blocked_reason)
        self.signals.backend_status_changed.emit(status)
        previous = self._last_status
        self._last_status = status
        for level, message in backend_transition_events(previous, status):
            self.signals.log_message.emit(level, message)


class RosBridge:
    """Own the ROS node and executor thread used by the Qt application."""

    def __init__(self):
        self.signals = RosSignals()
        self.node = ArmConsoleNode(self.signals)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self._spin,
            name='arm-console-ros-executor',
            daemon=True,
        )
        self._started = False

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    @property
    def joint_limits_degrees(self):
        """Expose limits without exposing the ROS node to widgets."""
        return self.node.joint_limits_degrees

    @property
    def gripper_pulses(self):
        """Expose configured OPEN and CLOSE pulses to the UI."""
        return (
            self.node.gripper_open_pulse_us,
            self.node.gripper_close_pulse_us,
        )

    def start(self):
        """Start processing ROS callbacks in the background."""
        if self._started:
            return
        self._started = True
        self.thread.start()

    def request_move(self, target_degrees):
        """Queue a six-axis MOVE request."""
        self.node.request_move(target_degrees)

    def request_stop(self):
        """Queue a STOP request."""
        self.node.request_stop()

    def request_disable(self):
        """Queue a DISABLE request."""
        self.node.request_disable()

    def request_preview(self, target_degrees):
        """Queue a target update for the isolated RViz preview model."""
        self.node.request_preview(target_degrees)

    def request_gripper(self, pulse_us):
        """Queue one gripper command."""
        self.node.request_gripper(pulse_us)

    def request_live_mode(self, enabled, baseline_degrees=None):
        """Explicitly arm or disarm the guarded LIVE FOLLOW session."""
        self.node.request_live_mode(enabled, baseline_degrees)

    def request_live_target(self, target_degrees):
        """Replace the desired live target; publishing remains rate-limited."""
        self.node.request_live_target(target_degrees)

    def request_live_heartbeat(self):
        """Refresh the GUI deadman heartbeat for an active live target."""
        self.node.request_live_heartbeat()

    def shutdown(self):
        """Stop callbacks and destroy the ROS node without blocking Qt."""
        if self._started:
            if self.node._live_armed:
                self.node.request_live_mode(False)
                self.node._live_disarmed_event.wait(timeout=1.0)
            self.executor.shutdown(timeout_sec=2.0)
            self.thread.join(timeout=2.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
