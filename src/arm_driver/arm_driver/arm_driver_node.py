import math
import os
import re
import threading
import time
from pathlib import Path

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from arm_driver.joint_mapping import ideal_rad_to_raw_deg
from arm_driver.joint_mapping import raw_deg_to_ideal_rad


START_ACK_TIMEOUT_SEC = 1.0
DEFAULT_COMMAND_ACK_TIMEOUT_SEC = 0.35
DEFAULT_JOINT_RESPONSE_TIMEOUT_SEC = 0.35
DEFAULT_SERIAL_READ_TIMEOUT_SEC = 0.05
DEFAULT_SERIAL_WRITE_TIMEOUT_SEC = 0.2
DEFAULT_START_SETTLE_DELAY_SEC = 0.5
DEFAULT_DISCONNECT_AFTER_FAILURES = 3
DEFAULT_RECONNECT_INITIAL_DELAY_SEC = 0.5
DEFAULT_RECONNECT_MAX_DELAY_SEC = 5.0
MAX_RECEIVE_BUFFER_BYTES = 4096
SERIAL_READ_CHUNK_BYTES = 256
# The firmware FIFO has 16 entries. Push() performs Put() and GetSpace() as
# separate calls, so the consumer can pop between them and a successful push
# can report 16 free entries. 255 is the explicit failure sentinel.
MAX_COMMAND_QUEUE_FREE_SLOTS = 16
MIN_LIVE_QUEUE_FREE_SLOTS = 15

DEFAULT_NORMAL_JOINT_SPEED = 30.0
DEFAULT_LIVE_JOINT_SPEED = 10.0
MAX_LIVE_JOINT_SPEED = 20.0
DEFAULT_LIVE_COMMAND_PERIOD_SEC = 0.1
DEFAULT_LIVE_COMMAND_MAX_AGE_SEC = 0.25
DEFAULT_LIVE_FEEDBACK_TIMEOUT_SEC = 0.35
DEFAULT_LIVE_WATCHDOG_TIMEOUT_SEC = 0.30
DEFAULT_LIVE_MAX_STEP_DEG = 1.0
DEFAULT_LIVE_ZERO_DELTA_DEG = 0.02
LIVE_FUTURE_STAMP_TOLERANCE_SEC = 0.10
MIN_LIVE_COMMAND_PERIOD_SEC = 0.02
MAX_LIVE_COMMAND_PERIOD_SEC = 0.50
MAX_LIVE_COMMAND_MAX_AGE_SEC = 1.0
MAX_LIVE_FEEDBACK_TIMEOUT_SEC = 2.0
MAX_LIVE_WATCHDOG_TIMEOUT_SEC = 1.0
MAX_LIVE_MAX_STEP_DEG = 5.0
DEFAULT_JOINT_MIN_RAD = (-3.14,) * 6
DEFAULT_JOINT_MAX_RAD = (3.14,) * 6
J6_INDEX = 5

FLOAT_TOKEN = (
    rb'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
)
JOINT_RESPONSE_PATTERN = re.compile(
    rb'ok[ \t]+'
    + rb'[ \t]+'.join([FLOAT_TOKEN] * 6)
    + rb'(?=ok|[\r\n])',
    re.IGNORECASE,
)
START_ACK_PATTERN = re.compile(
    rb'Started[ \t]+ok(?=ok|[\r\n])',
    re.IGNORECASE,
)
STOP_ACK_PATTERN = re.compile(
    rb'Stopped[ \t]+ok(?=ok|[\r\n])',
    re.IGNORECASE,
)
DISABLE_ACK_PATTERN = re.compile(
    rb'Disabled[ \t]+ok(?=ok|[\r\n])',
    re.IGNORECASE,
)
COMMAND_MODE_2_ACK_PATTERN = re.compile(
    rb'(?:ok[ \t]+)?Set[ \t]+command[ \t]+mode[ \t]+to[ \t]+'
    rb'\[2\](?=ok|[\r\n])',
    re.IGNORECASE,
)
QUEUE_ACK_PATTERN = re.compile(
    rb'(?:(?:^|[\r\n])[ \t]*(\d+)|ok(\d+))(?=[\r\n]|ok)',
    re.IGNORECASE,
)


class ArmResponseTimeout(TimeoutError):
    """Raised when a complete expected controller response does not arrive."""


def resolve_stable_serial_port(port_name):
    """Prefer a matching /dev/serial/by-id alias for a configured tty path."""
    configured = os.path.abspath(os.path.expanduser(str(port_name)))
    by_id_directory = Path('/dev/serial/by-id')

    if configured.startswith(f'{by_id_directory}{os.sep}'):
        return configured

    try:
        configured_target = os.path.realpath(configured)
        candidates = sorted(by_id_directory.iterdir())
    except OSError:
        return configured

    for candidate in candidates:
        try:
            if os.path.realpath(candidate) == configured_target:
                return str(candidate)
        except OSError:
            continue

    return configured


class ArmDriveNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')

        self.declare_parameter('use_mock', True)
        # Real motion requires explicit use_mock:=false and authority opt-in.
        self.declare_parameter('allow_motion_commands', False)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('poll_period', 0.1)
        self.declare_parameter(
            'mock_raw_degrees',
            [0.0, -75.0, 180.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('joint_min_rad', list(DEFAULT_JOINT_MIN_RAD))
        self.declare_parameter('joint_max_rad', list(DEFAULT_JOINT_MAX_RAD))
        self.declare_parameter(
            'serial_read_timeout',
            DEFAULT_SERIAL_READ_TIMEOUT_SEC,
        )
        self.declare_parameter(
            'serial_write_timeout',
            DEFAULT_SERIAL_WRITE_TIMEOUT_SEC,
        )
        self.declare_parameter(
            'joint_response_timeout',
            DEFAULT_JOINT_RESPONSE_TIMEOUT_SEC,
        )
        self.declare_parameter(
            'command_ack_timeout',
            DEFAULT_COMMAND_ACK_TIMEOUT_SEC,
        )
        self.declare_parameter(
            'start_settle_delay',
            DEFAULT_START_SETTLE_DELAY_SEC,
        )
        self.declare_parameter(
            'disconnect_after_failures',
            DEFAULT_DISCONNECT_AFTER_FAILURES,
        )
        self.declare_parameter(
            'reconnect_initial_delay',
            DEFAULT_RECONNECT_INITIAL_DELAY_SEC,
        )
        self.declare_parameter(
            'reconnect_max_delay',
            DEFAULT_RECONNECT_MAX_DELAY_SEC,
        )
        self.declare_parameter(
            'normal_joint_speed',
            DEFAULT_NORMAL_JOINT_SPEED,
        )
        self.declare_parameter(
            'live_joint_speed',
            DEFAULT_LIVE_JOINT_SPEED,
        )
        self.declare_parameter(
            'live_command_period',
            DEFAULT_LIVE_COMMAND_PERIOD_SEC,
        )
        self.declare_parameter(
            'live_command_max_age',
            DEFAULT_LIVE_COMMAND_MAX_AGE_SEC,
        )
        self.declare_parameter(
            'live_feedback_timeout',
            DEFAULT_LIVE_FEEDBACK_TIMEOUT_SEC,
        )
        self.declare_parameter(
            'live_watchdog_timeout',
            DEFAULT_LIVE_WATCHDOG_TIMEOUT_SEC,
        )
        self.declare_parameter(
            'live_max_step_deg',
            DEFAULT_LIVE_MAX_STEP_DEG,
        )
        self.declare_parameter(
            'live_zero_delta_deg',
            DEFAULT_LIVE_ZERO_DELTA_DEG,
        )

        self.use_mock = self.get_parameter('use_mock').value
        self.allow_motion_commands = self.get_parameter(
            'allow_motion_commands'
        ).value
        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.poll_period = float(self.get_parameter('poll_period').value)
        self.mock_raw_degrees = self.get_parameter(
            'mock_raw_degrees'
        ).value
        self.joint_min_rad = tuple(
            float(value)
            for value in self.get_parameter('joint_min_rad').value
        )
        self.joint_max_rad = tuple(
            float(value)
            for value in self.get_parameter('joint_max_rad').value
        )
        self._validate_joint_limit_configuration()
        self.serial_read_timeout_sec = max(
            0.001,
            float(self.get_parameter('serial_read_timeout').value),
        )
        self.serial_write_timeout_sec = max(
            0.001,
            float(self.get_parameter('serial_write_timeout').value),
        )
        self.joint_response_timeout_sec = max(
            self.serial_read_timeout_sec,
            float(self.get_parameter('joint_response_timeout').value),
        )
        self.command_ack_timeout_sec = max(
            self.serial_read_timeout_sec,
            float(self.get_parameter('command_ack_timeout').value),
        )
        self.start_settle_delay_sec = max(
            0.0,
            float(self.get_parameter('start_settle_delay').value),
        )
        self.start_ack_timeout_sec = START_ACK_TIMEOUT_SEC
        self.disconnect_after_failures = max(
            1,
            int(self.get_parameter('disconnect_after_failures').value),
        )
        self.reconnect_initial_delay_sec = max(
            0.1,
            float(self.get_parameter('reconnect_initial_delay').value),
        )
        self.reconnect_max_delay_sec = max(
            self.reconnect_initial_delay_sec,
            float(self.get_parameter('reconnect_max_delay').value),
        )
        self.normal_joint_speed = self._validated_speed_parameter(
            'normal_joint_speed'
        )
        self.live_joint_speed = self._validated_speed_parameter(
            'live_joint_speed',
            maximum=MAX_LIVE_JOINT_SPEED,
        )
        self._validate_speed_configuration()
        self.live_command_period_sec = float(
            self.get_parameter('live_command_period').value
        )
        self.live_command_max_age_sec = float(
            self.get_parameter('live_command_max_age').value
        )
        self.live_feedback_timeout_sec = float(
            self.get_parameter('live_feedback_timeout').value
        )
        self.live_watchdog_timeout_sec = float(
            self.get_parameter('live_watchdog_timeout').value
        )
        self.live_max_step_deg = float(
            self.get_parameter('live_max_step_deg').value
        )
        self.live_zero_delta_deg = float(
            self.get_parameter('live_zero_delta_deg').value
        )
        self._validate_live_configuration()

        self.serial_port = None
        self.active_serial_port_name = None
        self.preferred_serial_port_name = None
        self.serial_lock = threading.Lock()
        self.serial_rx_buffer = bytearray()
        self.current_ideal_radians = None
        self.last_feedback_monotonic = None
        self.controller_started = False
        self.consecutive_poll_failures = 0
        self.connection_state = 'MOCK' if self.use_mock else 'DISCONNECTED'
        self.minimum_command_stamp_ns = None
        self.reconnect_delay_sec = self.reconnect_initial_delay_sec
        self.next_reconnect_time = 0.0
        self.live_follow_active = False
        self.live_follow_tracking = False
        self.live_epoch_stamp_ns = None
        self.live_armed_monotonic = None
        self.live_last_message_stamp_ns = None
        self.live_last_heartbeat_stamp_ns = None
        self.live_last_heartbeat_monotonic = None
        self.live_last_send_monotonic = None
        self.live_last_send_feedback_monotonic = None
        self.live_last_motion_command = None
        self.pending_live_target = None
        self._last_live_state_published = None

        if self.use_mock:
            self.get_logger().info(
                'Mock mode enabled, serial port will not be opened'
            )
        else:
            with self.serial_lock:
                self._maybe_reconnect_locked()

        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
        ]

        self.raw_joint_state_publisher = self.create_publisher(
            JointState,
            'arm/raw_joint_states',
            10,
        )

        self.model_joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10,
        )

        self.joint_command_subscription = self.create_subscription(
            JointState,
            'arm/joint_command',
            self.joint_command_callback,
            10,
        )

        live_command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.live_joint_command_subscription = self.create_subscription(
            JointState,
            '/arm/live_joint_command',
            self.live_joint_command_callback,
            live_command_qos,
        )
        self.live_follow_heartbeat_subscription = self.create_subscription(
            Header,
            '/arm/live_follow_heartbeat',
            self.live_follow_heartbeat_callback,
            live_command_qos,
        )

        live_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.live_follow_state_publisher = self.create_publisher(
            Bool,
            '/arm/live_follow_state',
            live_state_qos,
        )

        self.timer = self.create_timer(
            self.poll_period,
            self.publish_joint_state,
        )

        live_timer_period = min(
            self.live_command_period_sec / 2.0,
            self.live_watchdog_timeout_sec / 3.0,
        )
        self.live_timer = self.create_timer(
            max(0.01, live_timer_period),
            self.process_live_follow,
        )

        self.disable_service = self.create_service(
            Trigger,
            '/arm/disable',
            self.disable_callback,
        )

        self.stop_service = self.create_service(
            Trigger,
            '/arm/stop',
            self.stop_callback,
        )

        self.live_follow_service = self.create_service(
            SetBool,
            '/arm/live_follow',
            self.live_follow_callback,
        )

        self._publish_live_follow_state(False)

        self.get_logger().info(
            f'Arm Driver Node started | '
            f'use_mock={self.use_mock} | '
            f'port={self.serial_port_name} | '
            f'baud={self.baud_rate} | '
            f'poll_period={self.poll_period}s'
        )

    def _validated_speed_parameter(self, parameter_name, maximum=100.0):
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or not 0.0 < value <= maximum:
            raise ValueError(
                f'{parameter_name} must be finite and in (0, {maximum}]'
            )
        return value

    def _validate_speed_configuration(self):
        if self.live_joint_speed > self.normal_joint_speed:
            raise ValueError(
                'live_joint_speed must not exceed normal_joint_speed'
            )

    def _validate_live_configuration(self):
        values = {
            'poll_period': self.poll_period,
            'live_command_period': self.live_command_period_sec,
            'live_command_max_age': self.live_command_max_age_sec,
            'live_feedback_timeout': self.live_feedback_timeout_sec,
            'live_watchdog_timeout': self.live_watchdog_timeout_sec,
            'live_max_step_deg': self.live_max_step_deg,
            'live_zero_delta_deg': self.live_zero_delta_deg,
        }
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')

        if not 0.0 < self.poll_period <= 1.0:
            raise ValueError('poll_period must be in (0, 1.0] seconds')
        if not (
            MIN_LIVE_COMMAND_PERIOD_SEC
            <= self.live_command_period_sec
            <= MAX_LIVE_COMMAND_PERIOD_SEC
        ):
            raise ValueError(
                'live_command_period must be in '
                f'[{MIN_LIVE_COMMAND_PERIOD_SEC}, '
                f'{MAX_LIVE_COMMAND_PERIOD_SEC}] seconds'
            )
        if not (
            self.live_command_period_sec
            <= self.live_command_max_age_sec
            <= MAX_LIVE_COMMAND_MAX_AGE_SEC
        ):
            raise ValueError(
                'live_command_max_age must be at least '
                'live_command_period and at most '
                f'{MAX_LIVE_COMMAND_MAX_AGE_SEC} seconds'
            )
        minimum_feedback_timeout = max(
            self.poll_period * 2.0,
            self.live_command_period_sec,
        )
        if not (
            minimum_feedback_timeout
            <= self.live_feedback_timeout_sec
            <= MAX_LIVE_FEEDBACK_TIMEOUT_SEC
        ):
            raise ValueError(
                'live_feedback_timeout must be at least two poll periods '
                f'and at most {MAX_LIVE_FEEDBACK_TIMEOUT_SEC} seconds'
            )
        if not (
            self.live_command_period_sec * 2.0
            <= self.live_watchdog_timeout_sec
            <= MAX_LIVE_WATCHDOG_TIMEOUT_SEC
        ):
            raise ValueError(
                'live_watchdog_timeout must be at least two live command '
                f'periods and at most {MAX_LIVE_WATCHDOG_TIMEOUT_SEC} seconds'
            )
        if self.live_command_max_age_sec > self.live_watchdog_timeout_sec:
            raise ValueError(
                'live_command_max_age must not exceed '
                'live_watchdog_timeout'
            )
        if not 0.0 < self.live_max_step_deg <= MAX_LIVE_MAX_STEP_DEG:
            raise ValueError(
                'live_max_step_deg must be in '
                f'(0, {MAX_LIVE_MAX_STEP_DEG}] degrees'
            )
        if not 0.0 <= self.live_zero_delta_deg < self.live_max_step_deg:
            raise ValueError(
                'live_zero_delta_deg must be non-negative and less than '
                'live_max_step_deg'
            )

    def _validate_joint_limit_configuration(self):
        if len(self.joint_min_rad) != 6 or len(self.joint_max_rad) != 6:
            raise ValueError(
                'joint_min_rad and joint_max_rad must each contain 6 values'
            )
        for index, (lower, upper) in enumerate(
            zip(self.joint_min_rad, self.joint_max_rad)
        ):
            if (
                not math.isfinite(lower)
                or not math.isfinite(upper)
                or lower >= upper
            ):
                raise ValueError(
                    f'Invalid configured ideal limits for joint{index + 1}: '
                    f'[{lower}, {upper}] rad'
                )

    def _ideal_target_within_limits(self, ideal_radians):
        if len(ideal_radians) != 6:
            return False
        return all(
            math.isfinite(value)
            and (
                index == J6_INDEX
                or lower - 1e-9 <= value <= upper + 1e-9
            )
            for index, (value, lower, upper) in enumerate(zip(
                ideal_radians,
                self.joint_min_rad,
                self.joint_max_rad,
            ))
        )

    @staticmethod
    def _format_joint_command(raw_degrees, speed):
        return (
            '&'
            + ','.join(f'{angle:.2f}' for angle in raw_degrees)
            + f',{speed:.2f}\n'
        )

    @staticmethod
    def _raw_motion_command_is_valid(command):
        """Validate the serialized six-axis motion command boundary."""
        if not isinstance(command, str):
            return False
        text = command.strip()
        if not text.startswith('&'):
            return False
        fields = text[1:].split(',')
        if len(fields) not in (6, 7):
            return False
        try:
            values = [float(value) for value in fields]
        except ValueError:
            return False
        if not all(math.isfinite(value) for value in values):
            return False
        return True

    def parse_joint_response(self, response_text):
        fields = response_text.split()

        if len(fields) != 7:
            raise ValueError(
                f'Expected 7 fields, but got {len(fields)}'
            )

        if fields[0].lower() != 'ok':
            raise ValueError(
                f'Expected response to start with "ok", but got {fields[0]!r}'
            )

        joint_degrees = [float(value) for value in fields[1:]]

        if not all(math.isfinite(value) for value in joint_degrees):
            raise ValueError('Joint response contains nan or infinity')

        return joint_degrees

    def validate_joint_command(self, ideal_radians):
        if len(ideal_radians) != 6:
            self.get_logger().warning(
                f'Expected 6 target joints, but got {len(ideal_radians)}'
            )
            return False

        if not all(math.isfinite(angle) for angle in ideal_radians):
            self.get_logger().warning(
                'Joint command contains nan or infinity'
            )
            return False

        if not self._ideal_target_within_limits(ideal_radians):
            self.get_logger().warning(
                'Joint command is outside configured ideal limits | '
                f'ideal_rad={ideal_radians}'
            )
            return False

        if self.current_ideal_radians is None:
            self.get_logger().warning(
                'Current joint position is unknown'
            )
            return False

        return True

    def joint_command_callback(self, msg):
        if not self.allow_motion_commands:
            self.get_logger().warning(
                'Rejected normal MOVE because motion commands are disabled'
            )
            return

        if self.live_follow_active:
            self.get_logger().warning(
                'Rejected normal MOVE while live follow is armed'
            )
            return

        if not self._command_stamp_is_current(msg.header.stamp):
            return

        if len(msg.name) != 6 or len(msg.position) != 6:
            self.get_logger().warning(
                'Normal target must contain exactly six joint names '
                'and positions'
            )
            return
        if len(set(msg.name)) != 6 or set(msg.name) != set(self.joint_names):
            self.get_logger().warning(
                'Normal target names must contain joint1..joint6 exactly once'
            )
            return

        position_by_name = dict(zip(msg.name, msg.position))
        ideal_radians = [
            float(position_by_name[joint_name])
            for joint_name in self.joint_names
        ]

        if not self.validate_joint_command(ideal_radians):
            return

        raw_degrees = ideal_rad_to_raw_deg(ideal_radians)
        command = self._format_joint_command(
            raw_degrees,
            self.normal_joint_speed,
        )

        self.get_logger().info(
            f'Joint command received | '
            f'ideal_rad={ideal_radians} | '
            f'raw_deg={raw_degrees} | '
            f'command={command.strip()}'
        )

        self.send_joint_command(command, command_stamp=msg.header.stamp)

    def _command_stamp_is_current(self, stamp):
        command_stamp_ns = (
            int(stamp.sec) * 1_000_000_000
            + int(stamp.nanosec)
        )

        if command_stamp_ns == 0:
            self.get_logger().warning(
                'Rejected joint command with a zero header timestamp'
            )
            return False

        minimum_stamp_ns = self.minimum_command_stamp_ns
        if (
            minimum_stamp_ns is not None
            and command_stamp_ns <= minimum_stamp_ns
        ):
            self.get_logger().warning(
                'Rejected queued joint command at or before the active '
                'safety barrier | '
                f'command_stamp_ns={command_stamp_ns} | '
                f'minimum_stamp_ns={minimum_stamp_ns}'
            )
            return False

        return True

    @staticmethod
    def _stamp_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _feedback_is_fresh(self, now=None):
        if self.current_ideal_radians is None:
            return False
        if self.use_mock:
            return True
        last_feedback = self.last_feedback_monotonic
        if last_feedback is None:
            return False
        if now is None:
            now = time.monotonic()
        return now - last_feedback <= self.live_feedback_timeout_sec

    def _publish_live_follow_state(self, active):
        active = bool(active)
        if self._last_live_state_published is active:
            return
        publisher = getattr(self, 'live_follow_state_publisher', None)
        if publisher is not None:
            message = Bool()
            message.data = active
            publisher.publish(message)
        self._last_live_state_published = active

    def _invalidate_live_follow(self, reason=None, publish_state=True):
        """Revoke motion authority, optionally deferring public safe state."""
        was_active = self.live_follow_active
        self.live_follow_active = False
        self.live_follow_tracking = False
        self.live_epoch_stamp_ns = None
        self.live_armed_monotonic = None
        self.live_last_message_stamp_ns = None
        self.live_last_heartbeat_stamp_ns = None
        self.live_last_heartbeat_monotonic = None
        self.live_last_send_monotonic = None
        self.live_last_send_feedback_monotonic = None
        self.live_last_motion_command = None
        self.pending_live_target = None
        if publish_state:
            self._publish_live_follow_state(False)
        if was_active and reason:
            self.get_logger().warning(
                f'Live follow disarmed: {reason}'
            )

    def _activate_live_follow(self):
        epoch_stamp_ns = self.get_clock().now().nanoseconds
        if epoch_stamp_ns <= 0:
            raise ValueError('ROS clock is not ready for a live epoch')
        self.live_follow_active = True
        self.live_follow_tracking = False
        self.live_epoch_stamp_ns = epoch_stamp_ns
        self.live_armed_monotonic = time.monotonic()
        self.live_last_message_stamp_ns = None
        self.live_last_heartbeat_stamp_ns = None
        self.live_last_heartbeat_monotonic = None
        self.live_last_send_monotonic = None
        self.live_last_send_feedback_monotonic = None
        self.live_last_motion_command = None
        self.pending_live_target = None
        self._publish_live_follow_state(True)

    def live_follow_heartbeat_callback(self, msg):
        """Refresh the live deadman without creating a movement target."""
        if not self.live_follow_active:
            return

        stamp_ns = self._stamp_nanoseconds(msg.stamp)
        if stamp_ns <= 0:
            self.get_logger().warning(
                'Ignored live heartbeat with a zero timestamp'
            )
            return
        if (
            self.live_epoch_stamp_ns is None
            or stamp_ns < self.live_epoch_stamp_ns
        ):
            self.get_logger().warning(
                'Ignored live heartbeat from an earlier live epoch'
            )
            return
        if (
            self.live_last_heartbeat_stamp_ns is not None
            and stamp_ns <= self.live_last_heartbeat_stamp_ns
        ):
            self.get_logger().warning(
                'Ignored out-of-order live heartbeat timestamp'
            )
            return

        now_stamp_ns = self.get_clock().now().nanoseconds
        age_ns = now_stamp_ns - stamp_ns
        if age_ns > int(self.live_command_max_age_sec * 1_000_000_000):
            self.get_logger().warning('Ignored stale live heartbeat')
            return
        if age_ns < -int(
            LIVE_FUTURE_STAMP_TOLERANCE_SEC * 1_000_000_000
        ):
            self.get_logger().warning(
                'Ignored live heartbeat timestamp too far in the future'
            )
            return

        self.live_last_heartbeat_stamp_ns = stamp_ns
        self.live_last_heartbeat_monotonic = time.monotonic()

    def live_joint_command_callback(self, msg):
        """Cache one fresh six-axis target; serial I/O is timer-driven."""
        if not self.allow_motion_commands:
            self.get_logger().warning(
                'Ignored live target because motion commands are disabled'
            )
            return

        if not self.live_follow_active:
            self.get_logger().warning(
                'Ignored live target because live follow is not armed'
            )
            return

        if len(msg.name) != 6 or len(msg.position) != 6:
            self.get_logger().warning(
                'Live target must contain exactly six joint names '
                'and positions'
            )
            return
        if len(set(msg.name)) != 6 or set(msg.name) != set(self.joint_names):
            self.get_logger().warning(
                'Live target names must contain joint1..joint6 exactly once'
            )
            return

        position_by_name = dict(zip(msg.name, msg.position))
        desired_radians = [
            float(position_by_name[joint_name])
            for joint_name in self.joint_names
        ]
        if not all(math.isfinite(value) for value in desired_radians):
            self.get_logger().warning(
                'Live target contains nan or infinity'
            )
            return

        stamp_ns = self._stamp_nanoseconds(msg.header.stamp)
        if stamp_ns <= 0:
            self.get_logger().warning(
                'Ignored live target with a zero header timestamp'
            )
            return

        epoch_stamp_ns = self.live_epoch_stamp_ns
        if epoch_stamp_ns is None or stamp_ns < epoch_stamp_ns:
            self.get_logger().warning(
                'Ignored live target from an earlier live epoch'
            )
            return
        if (
            self.live_last_message_stamp_ns is not None
            and stamp_ns <= self.live_last_message_stamp_ns
        ):
            self.get_logger().warning(
                'Ignored out-of-order live target timestamp'
            )
            return

        now_stamp_ns = self.get_clock().now().nanoseconds
        age_ns = now_stamp_ns - stamp_ns
        if age_ns > int(self.live_command_max_age_sec * 1_000_000_000):
            self.get_logger().warning('Ignored stale live target')
            return
        if age_ns < -int(
            LIVE_FUTURE_STAMP_TOLERANCE_SEC * 1_000_000_000
        ):
            self.get_logger().warning(
                'Ignored live target timestamp too far in the future'
            )
            return

        if self.current_ideal_radians is None:
            self.get_logger().warning(
                'Ignored live target because joint feedback is unavailable'
            )
            return

        if not self._ideal_target_within_limits(desired_radians):
            self.get_logger().warning(
                'Ignored live target outside configured ideal limits | '
                f'ideal_rad={desired_radians}'
            )
            return

        self.live_last_message_stamp_ns = stamp_ns
        self.live_follow_tracking = True
        self.pending_live_target = (
            stamp_ns,
            tuple(desired_radians),
        )

    def process_live_follow(self):
        """Enforce the live watchdog and send at most one latest setpoint."""
        if not self.live_follow_active:
            return

        now = time.monotonic()
        watchdog_reference = self.live_last_heartbeat_monotonic
        if watchdog_reference is None:
            watchdog_reference = self.live_armed_monotonic
        if (
            watchdog_reference is not None
            and now - watchdog_reference
            > self.live_watchdog_timeout_sec
        ):
            with self.serial_lock:
                self._stop_live_follow_locked('live command watchdog expired')
            return

        # Activation has a short grace window, but movement authority starts
        # only after this epoch receives its first valid deadman heartbeat.
        if self.live_last_heartbeat_monotonic is None:
            return
        if self.pending_live_target is None:
            return
        if (
            self.live_last_send_monotonic is not None
            and now - self.live_last_send_monotonic
            < self.live_command_period_sec
        ):
            return

        # Keep the latest six-axis desired position until it is replaced or
        # live mode is invalidated. A single slider event can therefore
        # converge over several feedback-bounded steps; the independent
        # heartbeat remains the sole deadman signal.
        pending_target = self.pending_live_target
        with self.serial_lock:
            self._send_live_target_locked(pending_target, now)

    def _send_live_target_locked(self, pending_target, now):
        if not self.allow_motion_commands:
            self._invalidate_live_follow(
                'motion commands are disabled by driver interlock'
            )
            return False

        if not self.live_follow_active:
            return False
        if self.live_last_heartbeat_monotonic is None:
            return False

        stamp_ns, desired_radians = pending_target
        if (
            self.live_epoch_stamp_ns is None
            or stamp_ns < self.live_epoch_stamp_ns
        ):
            return False
        if not self._feedback_is_fresh(now):
            self._stop_live_follow_locked('live feedback is not fresh')
            return False
        # Never advance twice from the same feedback sample. This keeps each
        # bounded target tied to measured state instead of building a queue of
        # speculative steps while the feedback timer has not advanced.
        if (
            not self.use_mock
            and self.live_last_send_feedback_monotonic is not None
            and self.last_feedback_monotonic
            <= self.live_last_send_feedback_monotonic
        ):
            return False

        current = list(self.current_ideal_radians)
        bounded_delta_degrees = [
            max(
                -self.live_max_step_deg,
                min(
                    self.live_max_step_deg,
                    math.degrees(desired - measured),
                ),
            )
            for desired, measured in zip(desired_radians, current)
        ]
        if all(
            abs(delta) <= self.live_zero_delta_deg
            for delta in bounded_delta_degrees
        ):
            self.live_last_send_monotonic = now
            return True

        target = [
            measured + math.radians(delta)
            for measured, delta in zip(current, bounded_delta_degrees)
        ]
        if not self._ideal_target_within_limits(target):
            self._stop_live_follow_locked(
                'bounded target is outside configured ideal limits'
            )
            return False
        raw_degrees = ideal_rad_to_raw_deg(target)

        command = self._format_joint_command(
            raw_degrees,
            self.live_joint_speed,
        )
        if command == self.live_last_motion_command:
            self.live_last_send_monotonic = now
            return True
        if self.use_mock:
            self.current_ideal_radians = target
            self.mock_raw_degrees = raw_degrees
            self.last_feedback_monotonic = now
            self.live_last_send_monotonic = now
            self.live_last_send_feedback_monotonic = (
                self.last_feedback_monotonic
            )
            self.live_last_motion_command = command
            self.get_logger().debug(f'[MOCK LIVE SEND] {command.strip()}')
            return True

        if (
            not self._serial_is_open()
            or self.connection_state != 'CONNECTED'
            or not self.controller_started
        ):
            self._stop_live_follow_locked(
                'controller is not ready for live motion',
                disconnect_after_stop=True,
            )
            return False

        try:
            self._write_locked(command.encode('ascii'))
            match, observed = self._read_matching_response_locked(
                QUEUE_ACK_PATTERN,
                self.command_ack_timeout_sec,
            )
        except (serial.SerialException, OSError) as error:
            self._handle_serial_exception_locked(
                'sending live target',
                error,
            )
            return False

        if match is None:
            self.get_logger().error(
                'Live queue acknowledgement timed out | '
                f'raw={observed!r}'
            )
            self._stop_live_follow_locked(
                'live queue acknowledgement timed out',
                disconnect_after_stop=True,
            )
            return False

        free_slots_text = match.group(1) or match.group(2)
        free_slots = int(free_slots_text)
        if not 0 <= free_slots <= MAX_COMMAND_QUEUE_FREE_SLOTS:
            self.get_logger().error(
                f'Invalid live queue acknowledgement: {free_slots}'
            )
            self._stop_live_follow_locked(
                'invalid live queue acknowledgement',
                disconnect_after_stop=True,
            )
            return False
        if free_slots < MIN_LIVE_QUEUE_FREE_SLOTS:
            self.get_logger().error(
                'Live command FIFO pressure detected; stopping instead of '
                f'executing stale targets | queue_free={free_slots}'
            )
            self._stop_live_follow_locked(
                'live command FIFO is backlogged'
            )
            return False

        self.live_last_send_monotonic = now
        self.live_last_send_feedback_monotonic = self.last_feedback_monotonic
        self.live_last_motion_command = command
        self.get_logger().debug(
            f'[SERIAL LIVE SEND] {command.strip()} | '
            f'queue_free={free_slots}'
        )
        return True

    def send_joint_command(self, command, command_stamp=None):
        if not self.allow_motion_commands:
            self.get_logger().warning(
                'Rejected normal MOVE because motion commands are disabled'
            )
            return False

        if self.live_follow_active:
            self.get_logger().error(
                'Cannot send normal joint command while live follow is armed'
            )
            return False

        if not self._raw_motion_command_is_valid(command):
            self.get_logger().error(
                'Rejected serialized MOVE: malformed six-axis target'
            )
            return False

        if self.use_mock:
            self.get_logger().info(f'[MOCK SEND] {command.strip()}')
            return True

        if not self._serial_is_open():
            self.get_logger().error(
                'Cannot send joint command: serial port is not open'
            )
            return False

        if (
            self.connection_state != 'CONNECTED'
            or self.current_ideal_radians is None
            or not self._feedback_is_fresh()
        ):
            self.get_logger().error(
                'Cannot send joint command: fresh arm feedback is unavailable'
            )
            return False

        try:
            with self.serial_lock:
                if not self._serial_is_open():
                    return False

                if (
                    self.connection_state != 'CONNECTED'
                    or self.current_ideal_radians is None
                    or not self._feedback_is_fresh()
                    or self.live_follow_active
                ):
                    return False

                if (
                    command_stamp is not None
                    and not self._command_stamp_is_current(command_stamp)
                ):
                    return False

                if not self.controller_started:
                    if not self._start_controller_locked():
                        return False
                    if self.start_settle_delay_sec > 0.0:
                        self.get_logger().info(
                            'Waiting '
                            f'{self.start_settle_delay_sec:.3f} s for '
                            'controller startup to settle before first MOVE'
                        )
                        time.sleep(self.start_settle_delay_sec)

                encoded_command = command.encode('ascii')
                self._write_locked(encoded_command)
                match, observed = self._read_matching_response_locked(
                    QUEUE_ACK_PATTERN,
                    self.command_ack_timeout_sec,
                )

                if match is None:
                    self.get_logger().error(
                        'MOVE queue acknowledgement timed out; '
                        'the command will not be replayed | '
                        f'raw={observed!r}'
                    )
                    self._disconnect_serial_locked()
                    return False

                free_slots_text = match.group(1) or match.group(2)
                free_slots = int(free_slots_text)
                if not 0 <= free_slots <= MAX_COMMAND_QUEUE_FREE_SLOTS:
                    self.get_logger().error(
                        'Arm controller rejected MOVE or returned an invalid '
                        f'queue acknowledgement: {free_slots}'
                    )
                    self._disconnect_serial_locked()
                    return False

            self.get_logger().info(
                f'[SERIAL SEND] {command.strip()} | '
                f'queue_free={free_slots}'
            )
            return True

        except (serial.SerialException, OSError) as error:
            with self.serial_lock:
                self._handle_serial_exception_locked(
                    'sending joint command',
                    error,
                )
            return False

    def _start_controller_locked(self, disconnect_on_failure=True):
        if not self.allow_motion_commands:
            self.controller_started = False
            self.get_logger().warning(
                'Rejected controller START because motion commands are '
                'disabled'
            )
            return False

        self._write_locked(b'!START\n')
        self.get_logger().info('[SERIAL SEND] !START')

        match, observed = self._read_matching_response_locked(
            START_ACK_PATTERN,
            self.start_ack_timeout_sec,
        )
        if match is not None:
            self.controller_started = True
            self.get_logger().info('[SERIAL RECEIVE] Started ok')
            return True

        self.controller_started = False
        self.get_logger().error(
            'Controller START acknowledgement timed out; '
            'joint command was not sent | '
            f'raw={observed!r}'
        )
        if disconnect_on_failure:
            self._disconnect_serial_locked()
        return False

    def _stop_controller_locked(self):
        self._write_locked(b'!STOP\n')
        self.controller_started = False
        self.get_logger().info('[SERIAL SEND] !STOP')
        match, observed = self._read_matching_response_locked(
            STOP_ACK_PATTERN,
            self.command_ack_timeout_sec,
        )
        if match is None:
            return False, observed
        self.get_logger().info('[SERIAL RECEIVE] Stopped ok')
        return True, observed

    def _set_interruptible_mode_locked(self):
        self._write_locked(b'#CMDMODE 2\n')
        self.get_logger().info('[SERIAL SEND] #CMDMODE 2')
        match, observed = self._read_matching_response_locked(
            COMMAND_MODE_2_ACK_PATTERN,
            self.command_ack_timeout_sec,
        )
        if match is None:
            return False, observed
        self.get_logger().info(
            '[SERIAL RECEIVE] Set command mode to [2]'
        )
        return True, observed

    def _stop_live_follow_locked(
        self,
        reason,
        disconnect_after_stop=False,
    ):
        """Invalidate live work first, then issue the firmware FIFO barrier."""
        self._advance_command_stamp_barrier()
        self._invalidate_live_follow(reason, publish_state=False)
        if self.use_mock:
            self.controller_started = False
            self._publish_live_follow_state(False)
            return True
        if not self._serial_is_open():
            self._publish_live_follow_state(False)
            return False

        try:
            stopped, observed = self._stop_controller_locked()
        except (serial.SerialException, OSError) as error:
            self._handle_serial_exception_locked(
                'stopping live follow',
                error,
            )
            return False

        if not stopped:
            self.get_logger().error(
                'Live safety STOP acknowledgement timed out; '
                f'controller state is unknown | raw={observed!r}'
            )
            self._disconnect_serial_locked()
            return False
        if disconnect_after_stop:
            self._disconnect_serial_locked()
        else:
            self._publish_live_follow_state(False)
        return True

    def live_follow_callback(self, request, response):
        """Arm or disarm bounded live-follow control."""
        if bool(request.data):
            return self._arm_live_follow(response)
        return self._disarm_live_follow(response)

    def _arm_live_follow(self, response):
        if not self.allow_motion_commands:
            self._invalidate_live_follow()
            response.success = False
            response.message = (
                'Motion commands are disabled by driver interlock'
            )
            return response

        if self.live_follow_active:
            response.success = True
            response.message = 'Live follow is already armed'
            return response

        self._invalidate_live_follow()
        if self.use_mock:
            joint_degrees = list(self.mock_raw_degrees)
            current_ideal_radians = raw_deg_to_ideal_rad(joint_degrees)
            if not self._ideal_target_within_limits(
                current_ideal_radians
            ):
                response.success = False
                response.message = (
                    'Mock feedback is outside configured ideal limits'
                )
                return response
            self.current_ideal_radians = current_ideal_radians
            self.last_feedback_monotonic = time.monotonic()
            self.controller_started = True
            try:
                self._activate_live_follow()
            except ValueError as error:
                self.controller_started = False
                response.success = False
                response.message = str(error)
                return response
            response.success = True
            response.message = (
                'Mock live follow armed in interruptible mode; no motion sent'
            )
            self.get_logger().info('[MOCK LIVE] STOP -> START -> CMDMODE 2')
            return response

        if not self._serial_is_open():
            response.success = False
            response.message = 'Serial port is not open'
            return response
        if (
            self.connection_state != 'CONNECTED'
            or not self._feedback_is_fresh()
        ):
            response.success = False
            response.message = 'Fresh arm feedback is required for live follow'
            return response

        with self.serial_lock:
            if (
                not self._serial_is_open()
                or self.connection_state != 'CONNECTED'
                or not self._feedback_is_fresh()
            ):
                response.success = False
                response.message = (
                    'Arm connection changed before live follow could be armed'
                )
                return response

            try:
                stopped, observed = self._stop_controller_locked()
                if not stopped:
                    raise ArmResponseTimeout(
                        'Initial live STOP acknowledgement timed out | '
                        f'raw={observed!r}'
                    )
                if not self._start_controller_locked(
                    disconnect_on_failure=False,
                ):
                    raise ArmResponseTimeout(
                        'Live START acknowledgement timed out'
                    )
                mode_set, observed = self._set_interruptible_mode_locked()
                if not mode_set:
                    raise ArmResponseTimeout(
                        'CMDMODE 2 acknowledgement timed out | '
                        f'raw={observed!r}'
                    )

                joint_degrees = self._query_joint_positions_locked()
                current_ideal_radians = raw_deg_to_ideal_rad(
                    joint_degrees
                )
                if not self._ideal_target_within_limits(
                    current_ideal_radians
                ):
                    raise ValueError(
                        'Current feedback is outside configured ideal limits; '
                        'live follow cannot establish a safe baseline'
                    )
                self.current_ideal_radians = current_ideal_radians
                self._record_poll_success_locked()
                self._activate_live_follow()
            except (ArmResponseTimeout, ValueError) as error:
                self.get_logger().error(
                    f'Failed to arm live follow: {error}'
                )
                self._stop_live_follow_locked(
                    'live arm transaction failed',
                    disconnect_after_stop=True,
                )
                response.success = False
                response.message = str(error)
                return response
            except (serial.SerialException, OSError) as error:
                self._handle_serial_exception_locked(
                    'arming live follow',
                    error,
                )
                response.success = False
                response.message = str(error)
                return response

        response.success = True
        response.message = (
            'Live follow armed; first target baseline is fresh feedback'
        )
        self.get_logger().info(
            'Live follow armed without sending a movement target'
        )
        return response

    def _disarm_live_follow(self, response):
        self._advance_command_stamp_barrier()
        self._invalidate_live_follow(
            'operator requested live exit',
            publish_state=False,
        )
        if self.use_mock:
            self.controller_started = False
            self._publish_live_follow_state(False)
            response.success = True
            response.message = 'Mock live follow stopped'
            self.get_logger().info('[MOCK LIVE] !STOP')
            return response
        if not self._serial_is_open():
            self._publish_live_follow_state(False)
            response.success = False
            response.message = (
                'Serial port is not open; live state is locked off'
            )
            return response

        with self.serial_lock:
            if not self._serial_is_open():
                self._publish_live_follow_state(False)
                response.success = False
                response.message = (
                    'Serial port closed; live state is locked off'
                )
                return response
            try:
                stopped, observed = self._stop_controller_locked()
            except (serial.SerialException, OSError) as error:
                self._handle_serial_exception_locked(
                    'exiting live follow',
                    error,
                )
                response.success = False
                response.message = str(error)
                return response
            if not stopped:
                response.success = False
                response.message = (
                    'Live exit STOP acknowledgement timed out; '
                    'controller state is unknown'
                )
                self.get_logger().error(
                    f'{response.message} | raw={observed!r}'
                )
                self._disconnect_serial_locked()
                return response

        self._publish_live_follow_state(False)
        response.success = True
        response.message = 'Live follow stopped and firmware FIFO cleared'
        return response

    def _write_locked(self, payload):
        if not self._serial_is_open():
            raise serial.SerialException('Serial port is not open')

        written = self.serial_port.write(payload)
        if written != len(payload):
            raise serial.SerialTimeoutException(
                f'Partial serial write: {written}/{len(payload)} bytes'
            )

    def _read_matching_response_locked(self, pattern, timeout_sec):
        deadline = time.monotonic() + timeout_sec

        while True:
            snapshot = bytes(self.serial_rx_buffer)
            match = pattern.search(snapshot)
            if match is not None:
                observed = snapshot[:match.end()].decode(
                    'utf-8',
                    errors='replace',
                )
                del self.serial_rx_buffer[:match.end()]
                return match, observed

            if time.monotonic() >= deadline:
                return None, snapshot.decode('utf-8', errors='replace')

            waiting = getattr(self.serial_port, 'in_waiting', 0)
            read_size = min(
                max(int(waiting), 1),
                SERIAL_READ_CHUNK_BYTES,
            )
            chunk = self.serial_port.read(read_size)
            if not chunk:
                continue

            self.serial_rx_buffer.extend(chunk)
            if len(self.serial_rx_buffer) > MAX_RECEIVE_BUFFER_BYTES:
                overflow = (
                    len(self.serial_rx_buffer) - MAX_RECEIVE_BUFFER_BYTES
                )
                del self.serial_rx_buffer[:overflow]

    def _query_joint_positions_locked(self):
        self._write_locked(b'#GETJPOS\n')
        match, observed = self._read_matching_response_locked(
            JOINT_RESPONSE_PATTERN,
            self.joint_response_timeout_sec,
        )

        if match is None:
            raise ArmResponseTimeout(
                'No valid GETJPOS response before deadline | '
                f'raw={observed!r}'
            )

        response_text = match.group(0).decode(
            'utf-8',
            errors='replace',
        )
        return self.parse_joint_response(response_text)

    def publish_joint_state(self):
        if self.use_mock:
            joint_degrees = list(self.mock_raw_degrees)
        else:
            with self.serial_lock:
                if not self._serial_is_open():
                    if not self._maybe_reconnect_locked():
                        return

                try:
                    joint_degrees = self._query_joint_positions_locked()
                except ArmResponseTimeout as error:
                    self._record_poll_failure_locked(str(error))
                    return
                except (serial.SerialException, OSError) as error:
                    self._handle_serial_exception_locked(
                        'polling joint state',
                        error,
                    )
                    return
                except ValueError as error:
                    self._record_poll_failure_locked(
                        f'Invalid joint response: {error}'
                    )
                    return

                ideal_joint_radians = raw_deg_to_ideal_rad(joint_degrees)
                self.current_ideal_radians = ideal_joint_radians
                self._record_poll_success_locked()

        raw_joint_radians = [
            math.radians(angle) for angle in joint_degrees
        ]
        if self.use_mock:
            ideal_joint_radians = raw_deg_to_ideal_rad(joint_degrees)
            self.current_ideal_radians = ideal_joint_radians
            self.last_feedback_monotonic = time.monotonic()

        stamp = self.get_clock().now().to_msg()

        raw_msg = JointState()
        raw_msg.header.stamp = stamp
        raw_msg.name = self.joint_names
        raw_msg.position = raw_joint_radians

        ideal_msg = JointState()
        ideal_msg.header.stamp = stamp
        ideal_msg.name = self.joint_names
        ideal_msg.position = ideal_joint_radians

        self.raw_joint_state_publisher.publish(raw_msg)
        self.model_joint_state_publisher.publish(ideal_msg)

    def _record_poll_success_locked(self):
        self.last_feedback_monotonic = time.monotonic()
        previous_state = self.connection_state
        recovered = previous_state != 'CONNECTED'
        if previous_state == 'PROBING':
            self._advance_command_stamp_barrier()
        self.connection_state = 'CONNECTED'
        self.consecutive_poll_failures = 0
        self.reconnect_delay_sec = self.reconnect_initial_delay_sec
        self.next_reconnect_time = 0.0

        if recovered:
            self.get_logger().info(
                'Arm controller feedback recovered; '
                'controller state is not assumed and no START or motion '
                'command was sent during recovery'
            )

    def _record_poll_failure_locked(self, reason):
        self.consecutive_poll_failures += 1
        failure_count = self.consecutive_poll_failures
        self.get_logger().warning(
            'Arm controller feedback failure '
            f'{failure_count}/{self.disconnect_after_failures}: {reason}'
        )

        if self.live_follow_active:
            stopped = self._stop_live_follow_locked(
                'first feedback poll failure during live follow'
            )
            if not stopped:
                return

        if failure_count >= self.disconnect_after_failures:
            self.get_logger().error(
                'Arm controller declared disconnected after '
                f'{failure_count} consecutive feedback failures'
            )
            self._disconnect_serial_locked()

    def _handle_serial_exception_locked(self, operation, error):
        self.get_logger().error(
            f'Serial error while {operation}: {error}'
        )
        self._invalidate_live_follow(
            f'serial error while {operation}',
            publish_state=False,
        )
        self._disconnect_serial_locked()

    def _serial_is_open(self):
        return (
            self.serial_port is not None
            and bool(getattr(self.serial_port, 'is_open', False))
        )

    def _maybe_reconnect_locked(self, now=None):
        if self._serial_is_open():
            return True

        if now is None:
            now = time.monotonic()

        if now < self.next_reconnect_time:
            return False

        self.connection_state = 'RECONNECTING'
        active_name = self.preferred_serial_port_name
        if active_name is None:
            active_name = resolve_stable_serial_port(self.serial_port_name)

        try:
            new_port = serial.Serial(
                port=active_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.serial_read_timeout_sec,
                write_timeout=self.serial_write_timeout_sec,
                exclusive=True,
            )
        except (serial.SerialException, OSError) as error:
            self.connection_state = 'DISCONNECTED'
            self._schedule_reconnect_locked(now)
            self.get_logger().warning(
                f'Cannot open arm serial port {active_name}: {error}; '
                f'retrying in {self.next_reconnect_time - now:.1f}s'
            )
            return False

        self.serial_port = new_port
        self.active_serial_port_name = active_name
        by_id_prefix = f'{Path("/dev/serial/by-id")}{os.sep}'
        if active_name.startswith(by_id_prefix):
            self.preferred_serial_port_name = active_name
        self.serial_rx_buffer.clear()
        self.controller_started = False
        self.current_ideal_radians = None
        self.last_feedback_monotonic = None
        self.consecutive_poll_failures = 0
        self.connection_state = 'PROBING'

        self.get_logger().info(
            f'Serial port opened for feedback probe: {active_name}; '
            'no START or motion command was sent'
        )
        return True

    def _schedule_reconnect_locked(self, now):
        self.next_reconnect_time = now + self.reconnect_delay_sec
        self.reconnect_delay_sec = min(
            self.reconnect_delay_sec * 2.0,
            self.reconnect_max_delay_sec,
        )

    def _disconnect_serial_locked(self):
        self._invalidate_live_follow(
            'arm serial connection lost',
            publish_state=False,
        )
        old_port = self.serial_port
        self.serial_port = None
        self.active_serial_port_name = None
        self.serial_rx_buffer.clear()
        self.controller_started = False
        self.current_ideal_radians = None
        self.last_feedback_monotonic = None
        self.consecutive_poll_failures = 0
        self.connection_state = 'DISCONNECTED'

        if old_port is not None:
            try:
                if getattr(old_port, 'is_open', False):
                    old_port.close()
            except (serial.SerialException, OSError) as error:
                self.get_logger().warning(
                    f'Failed to close arm serial port cleanly: {error}'
                )

        self._schedule_reconnect_locked(time.monotonic())
        self._publish_live_follow_state(False)

    def _advance_command_stamp_barrier(self):
        """Reject MOVE samples stamped before a safety action began."""
        now_ns = self.get_clock().now().nanoseconds
        existing_ns = self.minimum_command_stamp_ns or 0
        self.minimum_command_stamp_ns = max(existing_ns, now_ns)

    def disable_callback(self, request, response):
        del request
        self._advance_command_stamp_barrier()
        was_live = (
            self.live_follow_active
            or self._last_live_state_published is True
        )
        self._invalidate_live_follow(
            'DISABLE requested',
            publish_state=not was_live,
        )
        if self.use_mock:
            self.controller_started = False
            if was_live:
                self._publish_live_follow_state(False)
            response.success = True
            response.message = 'Mock controller disabled'
            self.get_logger().info('[MOCK SEND] !DISABLE')
            return response

        if was_live:
            with self.serial_lock:
                if not self._serial_is_open():
                    self._publish_live_follow_state(False)
                    response.success = False
                    response.message = (
                        'Serial port is not open; live state is locked off'
                    )
                    return response
                try:
                    stopped, observed = self._stop_controller_locked()
                except (serial.SerialException, OSError) as error:
                    self._handle_serial_exception_locked(
                        'stopping before DISABLE',
                        error,
                    )
                    response.success = False
                    response.message = str(error)
                    return response
                if not stopped:
                    response.success = False
                    response.message = (
                        'STOP before DISABLE timed out; controller state is '
                        'unknown'
                    )
                    self.get_logger().error(
                        f'{response.message} | raw={observed!r}'
                    )
                    self._disconnect_serial_locked()
                    return response
                self._publish_live_follow_state(False)

        return self._run_control_command(
            response=response,
            command=b'!DISABLE\n',
            label='DISABLE',
            ack_pattern=DISABLE_ACK_PATTERN,
            success_message='Controller disabled',
        )

    def stop_callback(self, request, response):
        del request
        self._advance_command_stamp_barrier()
        was_live = (
            self.live_follow_active
            or self._last_live_state_published is True
        )
        self._invalidate_live_follow(
            'STOP requested',
            publish_state=not was_live,
        )
        if self.use_mock:
            self.controller_started = False
            if was_live:
                self._publish_live_follow_state(False)
            response.success = True
            response.message = 'Mock controller stopped'
            self.get_logger().info('[MOCK SEND] !STOP')
            return response

        result = self._run_control_command(
            response=response,
            command=b'!STOP\n',
            label='STOP',
            ack_pattern=STOP_ACK_PATTERN,
            success_message='Controller stopped',
        )
        if was_live:
            # Success means STOP was acknowledged. Every failure path either
            # found the port already closed or disconnected it first.
            self._publish_live_follow_state(False)
        return result

    def _run_control_command(
        self,
        response,
        command,
        label,
        ack_pattern,
        success_message,
    ):
        if not self._serial_is_open():
            self.controller_started = False
            response.success = False
            response.message = 'Serial port is not open'
            return response

        try:
            with self.serial_lock:
                if not self._serial_is_open():
                    response.success = False
                    response.message = 'Serial port is not open'
                    return response

                self._write_locked(command)
                self.controller_started = False
                self.get_logger().info(f'[SERIAL SEND] !{label}')
                match, observed = self._read_matching_response_locked(
                    ack_pattern,
                    self.command_ack_timeout_sec,
                )

                if match is None:
                    response.success = False
                    response.message = (
                        f'{label} acknowledgement timed out; '
                        'controller state is unknown'
                    )
                    self.get_logger().error(
                        f'{response.message} | raw={observed!r}'
                    )
                    self._disconnect_serial_locked()
                    return response

            response.success = True
            response.message = success_message
            self.get_logger().info(
                f'[SERIAL RECEIVE] {match.group(0).decode("ascii")}'
            )

        except (serial.SerialException, OSError) as error:
            with self.serial_lock:
                self._handle_serial_exception_locked(
                    f'sending {label}',
                    error,
                )
            response.success = False
            response.message = str(error)

        return response

    def close_serial_port(self):
        if self.use_mock:
            was_live = (
                self.live_follow_active
                or self._last_live_state_published is True
            )
            if was_live:
                self._advance_command_stamp_barrier()
            self._invalidate_live_follow(
                'driver shutdown',
                publish_state=not was_live,
            )
            self.controller_started = False
            if was_live:
                self._publish_live_follow_state(False)
            return

        with self.serial_lock:
            was_live = (
                self.live_follow_active
                or self._last_live_state_published is True
            )
            should_stop = self.live_follow_active or self.controller_started
            if was_live:
                self._advance_command_stamp_barrier()
            self._invalidate_live_follow(
                'driver shutdown',
                publish_state=not was_live,
            )
            if should_stop and self._serial_is_open():
                try:
                    stopped, observed = self._stop_controller_locked()
                    if not stopped:
                        self.get_logger().warning(
                            'Shutdown STOP was not confirmed | '
                            f'raw={observed!r}'
                        )
                except (serial.SerialException, OSError) as error:
                    self.get_logger().warning(
                        f'Shutdown STOP failed: {error}'
                    )

            old_port = self.serial_port
            self.serial_port = None
            self.active_serial_port_name = None
            self.serial_rx_buffer.clear()
            self.controller_started = False
            self.current_ideal_radians = None
            self.last_feedback_monotonic = None
            self.connection_state = 'DISCONNECTED'

            if old_port is not None and getattr(old_port, 'is_open', False):
                try:
                    old_port.close()
                except (serial.SerialException, OSError) as error:
                    self.get_logger().warning(
                        f'Failed to close arm serial port: {error}'
                    )
            if was_live:
                self._publish_live_follow_state(False)


def _shutdown_node(node):
    """Run every shutdown stage even if another SIGINT arrives mid-cleanup."""
    for action in (
        node.close_serial_port,
        node.destroy_node,
        rclpy.try_shutdown,
    ):
        try:
            action()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriveNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _shutdown_node(node)


if __name__ == '__main__':
    main()
