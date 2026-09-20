"""Deprecated tag-demo implementation, retained for historical offline tests.

The visual_grasp executable now routes to tabletop_grasp.controller. This
module cannot unlock hardware and is not part of the normal grasping stack.
"""

import math
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from dual_camera_foundation.visual_grasp_safety import (
    goal_reached,
    point_in_workspace,
    stable_median_target,
    validate_gripper_configuration,
)
from dual_camera_foundation.visual_ik_node import (
    RobotKinematics,
    axis_angle_rotation,
    quaternion_to_rotation,
)
from gripper_driver.gripper_geometry import pulse_to_tcp_joint_position_m


BASE_LINK = 'base_link'
TIP_LINK = 'tcp'

JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]

# Hardware remains opt-in through execute_hardware plus the workspace and
# gripper confirmations. Each stage sends one full MOVE goal and waits for
# feedback, so the controller FIFO is not flooded with repeated commands.
HARDWARE_EXECUTION_UNLOCKED = False

GRASP_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])


def wrap_to_pi(angle):
    """Normalize an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def shortest_angular_distance(source, target):
    """Return the signed shortest rotation from source to target."""
    return wrap_to_pi(float(target) - float(source))


def square_symmetry_candidates(nominal_roll):
    """Return the four equivalent grasp-axis directions for a square object."""
    return tuple(
        wrap_to_pi(nominal_roll + index * math.pi / 2.0)
        for index in range(4)
    )


def circular_mean(angles):
    """Return a wrap-safe mean angle, or None for a degenerate sample set."""
    values = np.asarray(angles, dtype=np.float64)
    if values.size == 0 or not np.all(np.isfinite(values)):
        return None
    result = math.atan2(
        float(np.mean(np.sin(values))), float(np.mean(np.cos(values)))
    )
    return result if math.isfinite(result) else None


def yaw_from_base_vector(vector):
    """Return base XY yaw for a non-degenerate planar direction."""
    values = np.asarray(vector, dtype=np.float64)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError('grasp direction must be a finite 3-vector')
    if math.hypot(values[0], values[1]) < 1.0e-8:
        raise ValueError('grasp direction is not planar enough for top grasp')
    return math.atan2(float(values[1]), float(values[0]))


def object_yaw_from_tag_rotation(rotation):
    """Extract only the top-face direction, not full AprilTag rotation."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError('AprilTag rotation must be a finite 3x3 matrix')
    # AprilTag +X identifies one edge of the square.  For a top grasp its
    # projection on base XY is the only orientation component we need.
    return yaw_from_base_vector(matrix @ np.array([1.0, 0.0, 0.0]))


def joint_distance_cost(kinematics, candidate, current):
    """Euclidean joint-space cost with wrapped continuous-joint distances."""
    delta = (
        np.asarray(candidate, dtype=np.float64)
        - np.asarray(current, dtype=np.float64)
    )
    for index in kinematics.continuous_joint_indices:
        delta[index] = shortest_angular_distance(
            current[index], candidate[index]
        )
    return float(np.linalg.norm(delta))


class VisualGraspController(Node):
    """Lock a stable target, preview IK, and require opt-in for hardware."""

    def __init__(self):
        super().__init__('visual_grasp_controller')

        self.declare_parameter('execute_hardware', False)
        self.declare_parameter('workspace_bounds_confirmed', False)
        self.declare_parameter('stable_sample_count', 7)
        self.declare_parameter('stability_span_m', 0.015)
        self.declare_parameter(
            'workspace_min_m',
            [-0.55, -0.55, 0.0],
        )
        self.declare_parameter(
            'workspace_max_m',
            [0.55, 0.55, 0.65],
        )
        self.declare_parameter('arrival_tolerance_m', 0.018)
        self.declare_parameter('ik_residual_tolerance_m', 0.003)
        self.declare_parameter('ik_rotation_tolerance_deg', 2.0)
        self.declare_parameter('target_timeout_sec', 0.6)
        self.declare_parameter('joint_timeout_sec', 0.6)
        self.declare_parameter('gripper_open_pulse_us', 600)
        self.declare_parameter('gripper_close_pulse_us', 1300)
        self.declare_parameter('grasp_offset_m', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_axis_offset_deg', 0.0)
        self.declare_parameter(
            'home_joint_degrees',
            [0.0, 75.0, -90.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('home_tolerance_deg', 2.0)

        self.execute_hardware = bool(
            self.get_parameter('execute_hardware').value
        )
        self.workspace_bounds_confirmed = bool(
            self.get_parameter('workspace_bounds_confirmed').value
        )
        self.stable_sample_count = int(
            self.get_parameter('stable_sample_count').value
        )
        self.stability_span_m = float(
            self.get_parameter('stability_span_m').value
        )
        self.workspace_min = np.asarray(
            self.get_parameter('workspace_min_m').value,
            dtype=np.float64,
        )
        self.workspace_max = np.asarray(
            self.get_parameter('workspace_max_m').value,
            dtype=np.float64,
        )
        self.arrival_tolerance_m = float(
            self.get_parameter('arrival_tolerance_m').value
        )
        self.ik_residual_tolerance_m = float(
            self.get_parameter('ik_residual_tolerance_m').value
        )
        self.ik_rotation_tolerance_deg = float(
            self.get_parameter('ik_rotation_tolerance_deg').value
        )
        self.target_timeout_sec = float(
            self.get_parameter('target_timeout_sec').value
        )
        self.joint_timeout_sec = float(
            self.get_parameter('joint_timeout_sec').value
        )
        self.open_pulse = int(
            self.get_parameter('gripper_open_pulse_us').value
        )
        self.close_pulse = int(
            self.get_parameter('gripper_close_pulse_us').value
        )
        self.grasp_offset = np.asarray(
            self.get_parameter('grasp_offset_m').value,
            dtype=np.float64,
        )
        self.grasp_axis_offset_deg = float(
            self.get_parameter('grasp_axis_offset_deg').value
        )
        self.home_joints = np.radians(np.asarray(
            self.get_parameter('home_joint_degrees').value,
            dtype=np.float64,
        ))
        self.home_tolerance = math.radians(float(
            self.get_parameter('home_tolerance_deg').value
        ))
        self._validate_parameters()

        self.control_period = 0.2
        # The current D405-only target is near the vertical-reach boundary.
        # A 40 mm staged clearance is IK-valid while 70/100 mm is not.
        self.z_offsets = [0.04, 0.02, 0.00]

        # This defines the required top-down approach only.  The yaw/roll is
        # selected from the observed tag direction after the target is locked.
        self.top_grasp_rotation = quaternion_to_rotation(
            0.724,
            0.0,
            0.0,
            0.690,
        )
        self.top_grasp_axis_yaw = yaw_from_base_vector(
            self.top_grasp_rotation @ GRASP_AXIS_LOCAL
        )
        # Right multiplication rotates about the TCP-local approach axis.
        # Measure its sign in base yaw rather than assuming J6 equals TCP roll.
        probe = self.top_grasp_rotation @ axis_angle_rotation(
            [0.0, -1.0, 0.0], 1.0e-4
        )
        probe_yaw = yaw_from_base_vector(probe @ GRASP_AXIS_LOCAL)
        self.roll_to_base_yaw_sign = 1.0 if shortest_angular_distance(
            self.top_grasp_axis_yaw, probe_yaw
        ) > 0.0 else -1.0
        self.locked_target_rotation = None
        self.locked_roll_selection = None

        share = get_package_share_directory('arm_description')
        urdf_path = share + '/urdf/dummy-v2.urdf'
        self.kinematics = RobotKinematics(
            urdf_path,
            BASE_LINK,
            TIP_LINK,
        )
        self.kinematics.set_passive_joint_position(
            'tcp_extension_joint',
            pulse_to_tcp_joint_position_m(self.open_pulse),
        )

        self.target_samples = deque(maxlen=self.stable_sample_count)
        self.target_yaw_samples = deque(maxlen=self.stable_sample_count)
        self.last_target_sample_time = None
        self.locked_target = None
        self.locked_target_time = None

        self.latest_joints = None
        self.latest_joints_time = None

        self.state = 'WAIT_TARGET'
        self.level = 0
        self.state_start_time = time.monotonic()
        self.solution_sequence = 0
        self.stop_requested = False
        self.command_in_flight = None
        self.last_wait_status_time = 0.0

        self.preview_publisher = self.create_publisher(
            JointState,
            '/perception/dry_run/visual_grasp_solution',
            10,
        )

        self.arm_publisher = None
        self.gripper_publisher = None
        self.stop_client = None
        if self.execute_hardware:
            self.arm_publisher = self.create_publisher(
                JointState,
                '/arm/joint_command',
                10,
            )
            self.gripper_publisher = self.create_publisher(
                Int32,
                '/gripper/pulse_us',
                10,
            )
            self.stop_client = self.create_client(Trigger, '/arm/stop')

        self.create_subscription(
            PoseStamped,
            '/perception/target_pose_base',
            self.target_callback,
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10,
        )
        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        mode = (
            'HARDWARE ENABLED (EXPERIMENTAL)'
            if self.execute_hardware
            else 'DRY-RUN: NO ARM OR GRIPPER COMMAND PUBLISHERS'
        )
        self.get_logger().warning(
            '\n'
            '========================================\n'
            f'VISUAL GRASP MODE: {mode}\n'
            '========================================\n'
            'A target is locked from a stable median before approach.\n'
            'Later vision updates are ignored to prevent occlusion drift.\n'
            'The tcp offset, fixed grasp orientation, workspace bounds, and\n'
            'Astra-to-base calibration still require on-robot validation.'
        )

    def _validate_parameters(self):
        positive_values = {
            'stability_span_m': self.stability_span_m,
            'arrival_tolerance_m': self.arrival_tolerance_m,
            'ik_residual_tolerance_m': self.ik_residual_tolerance_m,
            'ik_rotation_tolerance_deg': self.ik_rotation_tolerance_deg,
            'target_timeout_sec': self.target_timeout_sec,
            'joint_timeout_sec': self.joint_timeout_sec,
            'home_tolerance_deg': math.degrees(self.home_tolerance),
        }
        if self.stable_sample_count < 3:
            raise ValueError('stable_sample_count must be at least 3')
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if (
            self.workspace_min.shape != (3,)
            or self.workspace_max.shape != (3,)
            or not np.all(np.isfinite(self.workspace_min))
            or not np.all(np.isfinite(self.workspace_max))
            or not np.all(self.workspace_min < self.workspace_max)
        ):
            raise ValueError(
                'workspace bounds must be finite 3-vectors with min < max'
            )
        if (
            self.grasp_offset.shape != (3,)
            or not np.all(np.isfinite(self.grasp_offset))
        ):
            raise ValueError('grasp_offset_m must be a finite 3-vector')
        if not math.isfinite(self.grasp_axis_offset_deg):
            raise ValueError('grasp_axis_offset_deg must be finite')
        if (
            self.home_joints.shape != (6,)
            or not np.all(np.isfinite(self.home_joints))
        ):
            raise ValueError(
                'home_joint_degrees must contain six finite values'
            )
        if self.execute_hardware and not self.workspace_bounds_confirmed:
            raise ValueError(
                'execute_hardware requires workspace_bounds_confirmed=true '
                'and physically verified workspace_min_m/workspace_max_m'
            )
        validated_pulses = validate_gripper_configuration(
            self.open_pulse,
            self.close_pulse,
        )
        self.open_pulse, self.close_pulse = validated_pulses
        if self.execute_hardware and not HARDWARE_EXECUTION_UNLOCKED:
            raise ValueError(
                'This legacy tag grasp demo is retired and hardware execution is disabled. '
                'Use tabletop_grasp for perception and planning only.'
            )

    def _warn_throttled(self, message):
        self.get_logger().warning(
            message,
            throttle_duration_sec=5.0,
        )

    def _reset_target_sampling(self):
        self.target_samples.clear()
        self.target_yaw_samples.clear()
        self.last_target_sample_time = None
        self.locked_target = None
        self.locked_target_time = None
        self.locked_target_rotation = None
        self.locked_roll_selection = None

    def _target_stamp_is_fresh(self, message):
        if (
            message.header.stamp.sec == 0
            and message.header.stamp.nanosec == 0
        ):
            return False
        try:
            source_time = Time.from_msg(message.header.stamp)
            age_sec = (
                self.get_clock().now().nanoseconds
                - source_time.nanoseconds
            ) / 1.0e9
        except (OverflowError, TypeError, ValueError):
            return False
        return -0.1 <= age_sec <= self.target_timeout_sec

    def target_callback(self, message):
        if self.state != 'WAIT_TARGET' or self.locked_target is not None:
            return

        if message.header.frame_id.strip() != BASE_LINK:
            self._reset_target_sampling()
            self._warn_throttled(
                f'Rejecting target in frame {message.header.frame_id!r}; '
                f'expected {BASE_LINK!r}'
            )
            return
        if not self._target_stamp_is_fresh(message):
            self._reset_target_sampling()
            self._warn_throttled(
                'Rejecting target with a missing, stale, or future timestamp'
            )
            return

        point = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float64,
        )
        try:
            tag_rotation = quaternion_to_rotation(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
            object_yaw = object_yaw_from_tag_rotation(tag_rotation)
        except ValueError as error:
            self._reset_target_sampling()
            self._warn_throttled(f'Rejecting target orientation: {error}')
            return
        if not point_in_workspace(
            point,
            self.workspace_min,
            self.workspace_max,
        ):
            self._reset_target_sampling()
            self._warn_throttled(
                f'Rejecting non-finite or out-of-workspace target {point}'
            )
            return

        now = time.monotonic()
        if (
            self.last_target_sample_time is not None
            and now - self.last_target_sample_time > self.target_timeout_sec
        ):
            self.target_samples.clear()
            self.target_yaw_samples.clear()
        self.last_target_sample_time = now
        self.target_samples.append(point)
        self.target_yaw_samples.append(object_yaw)

        target, span = stable_median_target(
            list(self.target_samples),
            self.stable_sample_count,
            self.stability_span_m,
            self.workspace_min,
            self.workspace_max,
        )
        if target is None:
            if len(self.target_samples) >= self.stable_sample_count:
                self._warn_throttled(
                    'Target is not stable enough to lock '
                    f'(span={span * 1000.0:.1f} mm, '
                    f'limit={self.stability_span_m * 1000.0:.1f} mm)'
                )
            return

        grasp_target = target + self.grasp_offset
        if not point_in_workspace(
            grasp_target,
            self.workspace_min,
            self.workspace_max,
        ):
            self._reset_target_sampling()
            self._warn_throttled(
                'Rejecting target because Tag-to-grasp offset produces an '
                f'out-of-workspace point: {grasp_target}'
            )
            return

        self.locked_target = grasp_target.copy()
        self.locked_target_time = now
        locked_object_yaw = circular_mean(self.target_yaw_samples)
        if locked_object_yaw is None:
            self._reset_target_sampling()
            self._warn_throttled(
                'Rejecting target because its yaw samples are invalid'
            )
            return
        self.locked_object_yaw = locked_object_yaw
        self.get_logger().info(
            'TARGET LOCKED FROM STABLE MEDIAN\n'
            f'  samples = {self.stable_sample_count}\n'
            f'  span = {span * 1000.0:.1f} mm\n'
            '  tag_xyz = '
            f'[{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}] m\n'
            '  grasp_offset = '
            f'{np.round(self.grasp_offset, 4).tolist()} m\n'
            '  grasp_xyz = '
            f'{np.round(self.locked_target, 4).tolist()} m\n'
            f'  object_yaw={math.degrees(self.locked_object_yaw):.1f} deg '
            '(base XY, from AprilTag +X)'
        )

    def joint_callback(self, message):
        joint_map = dict(zip(message.name, message.position))
        if not all(name in joint_map for name in JOINT_NAMES):
            return
        joints = np.array(
            [joint_map[name] for name in JOINT_NAMES],
            dtype=np.float64,
        )
        if not self.kinematics.joints_within_limits(joints):
            self.latest_joints = None
            self.latest_joints_time = None
            self._warn_throttled(
                'Rejecting non-finite or out-of-limit joint feedback'
            )
            return
        self.latest_joints = joints
        self.latest_joints_time = time.monotonic()

    def joints_are_fresh(self):
        return bool(
            self.latest_joints is not None
            and self.latest_joints_time is not None
            and time.monotonic() - self.latest_joints_time
            <= self.joint_timeout_sec
        )

    def _request_stop(self, reason):
        if (
            not self.execute_hardware
            or self.stop_client is None
            or self.stop_requested
        ):
            return
        self.stop_requested = True
        if not self.stop_client.service_is_ready():
            self.get_logger().error(
                f'Cannot request /arm/stop after fault: {reason}; '
                'service is unavailable'
            )
            return
        self.stop_client.call_async(Trigger.Request())
        self.get_logger().error(f'/arm/stop requested: {reason}')

    def _fault(self, reason):
        if self.state == 'FAULT':
            return
        self.state = 'FAULT'
        self.get_logger().error(f'VISUAL GRASP FAULT: {reason}')
        self._request_stop(reason)

    def publish_gripper(self, pulse, action):
        if hasattr(self, 'kinematics'):
            self.kinematics.set_passive_joint_position(
                'tcp_extension_joint',
                pulse_to_tcp_joint_position_m(pulse),
            )
        if not self.execute_hardware:
            self.get_logger().info(
                f'[DRY-RUN] gripper {action}: {pulse} us NOT PUBLISHED'
            )
            return True
        if self.gripper_publisher is None:
            self._fault('gripper publisher is unavailable')
            return False
        message = Int32()
        message.data = int(pulse)
        self.gripper_publisher.publish(message)
        self.get_logger().warning(
            f'[HARDWARE] gripper {action}: {pulse} us published'
        )
        return True

    def _ik_result_is_usable(
        self,
        success,
        solution,
        position_error,
        approach_error_deg,
        full_rotation_error_deg,
    ):
        """Accept a bounded near-solution when the solver hits its limit."""
        if (
            np.asarray(solution).shape != (6,)
            or not np.all(np.isfinite(solution))
            or not math.isfinite(position_error)
            or not math.isfinite(approach_error_deg)
            or not math.isfinite(full_rotation_error_deg)
        ):
            return False
        if success:
            return True
        return bool(
            position_error <= self.ik_residual_tolerance_m
            and approach_error_deg <= self.ik_rotation_tolerance_deg
            and full_rotation_error_deg <= self.ik_rotation_tolerance_deg
        )

    def _target_rotation_for_grasp_yaw(self, grasp_yaw):
        """Build a top-down TCP rotation whose grasp axis has base yaw."""
        roll_delta = shortest_angular_distance(
            self.top_grasp_axis_yaw, grasp_yaw
        ) / self.roll_to_base_yaw_sign
        return self.top_grasp_rotation @ axis_angle_rotation(
            [0.0, -1.0, 0.0], roll_delta
        )

    def _select_square_grasp_orientation(self):
        """Solve all square-equivalent TCP rotations using the current seed."""
        if self.locked_target is None or not self.joints_are_fresh():
            return False
        nominal_roll = wrap_to_pi(
            self.locked_object_yaw + math.radians(self.grasp_axis_offset_deg)
        )
        candidate_rolls = square_symmetry_candidates(nominal_roll)
        current_tcp = self.kinematics.forward(self.latest_joints)[:3, :3]
        current_tcp_roll = yaw_from_base_vector(current_tcp @ GRASP_AXIS_LOCAL)
        pregrasp_position = self.locked_target.copy()
        pregrasp_position[2] += self.z_offsets[0]
        valid_candidates = []
        for index, roll in enumerate(candidate_rolls):
            target_rotation = self._target_rotation_for_grasp_yaw(roll)
            result = self.kinematics.solve(
                pregrasp_position, target_rotation, self.latest_joints
            )
            (
                success,
                solution,
                position_error,
                approach_error_deg,
                full_rotation_error_deg,
            ) = result
            solution = np.asarray(solution, dtype=np.float64)
            if (
                not self._ik_result_is_usable(*result)
                or not self.kinematics.joints_within_limits(solution)
            ):
                continue
            cost = joint_distance_cost(
                self.kinematics, solution, self.latest_joints
            )
            valid_candidates.append(
                (
                    cost, index, roll, target_rotation, solution,
                    position_error,
                    approach_error_deg,
                    full_rotation_error_deg,
                )
            )
        if not valid_candidates:
            self._fault(
                'No square-symmetry target rotation has a valid pregrasp '
                'IK solution'
            )
            return False
        valid_candidates.sort(key=lambda item: item[0])
        (
            cost, index, selected_roll, target_rotation, solution,
            _position_error, _approach_error_deg, _full_rotation_error_deg,
        ) = valid_candidates[0]
        self.locked_target_rotation = target_rotation
        self.locked_roll_selection = {
            'nominal_roll': nominal_roll,
            'candidate_rolls': candidate_rolls,
            'selected_index': index,
            'selected_roll': selected_roll,
            'cost': cost,
            'solved_j6': float(solution[5]),
            'current_tcp_roll': current_tcp_roll,
        }
        candidates_text = ', '.join(
            f'{math.degrees(value):.1f}' for value in candidate_rolls
        )
        self.get_logger().info(
            'SQUARE-SYMMETRY GRASP ORIENTATION LOCKED\n'
            f'  object_yaw={math.degrees(self.locked_object_yaw):.1f} deg\n'
            f'  nominal_grasp_roll={math.degrees(nominal_roll):.1f} deg\n'
            f'  symmetry_candidates=[{candidates_text}] deg\n'
            f'  current_tcp_roll={math.degrees(current_tcp_roll):.1f} deg\n'
            f'  current_j6={math.degrees(self.latest_joints[5]):.1f} deg\n'
            f'  selected_candidate={index}\n'
            f'  selected_roll={math.degrees(selected_roll):.1f} deg\n'
            f'  solved_j6={math.degrees(solution[5]):.1f} deg\n'
            f'  joint_distance_cost={cost:.4f} rad'
        )
        return True

    def send_cartesian_goal(self, desired_position):
        desired_position = np.asarray(
            desired_position,
            dtype=np.float64,
        )
        if not point_in_workspace(
            desired_position,
            self.workspace_min,
            self.workspace_max,
        ):
            self._fault(
                f'desired TCP point is outside workspace: {desired_position}'
            )
            return None
        if not self.joints_are_fresh():
            self._fault('joint feedback is stale')
            return None
        if self.locked_target_rotation is None:
            self._fault('No locked square-symmetry grasp orientation')
            return None

        (
            success,
            solution,
            ik_position_error,
            ik_approach_error_deg,
            ik_full_rotation_error_deg,
        ) = self.kinematics.solve(
            desired_position,
            self.locked_target_rotation,
            self.latest_joints,
        )
        solution = np.asarray(solution, dtype=np.float64)
        if not self._ik_result_is_usable(
            success,
            solution,
            ik_position_error,
            ik_approach_error_deg,
            ik_full_rotation_error_deg,
        ):
            self._fault(
                'IK failed or returned non-finite output '
                f'(position={ik_position_error}, '
                f'approach={ik_approach_error_deg}, '
                f'full_rotation={ik_full_rotation_error_deg})'
            )
            return None
        if not success:
            self.get_logger().warning(
                'Using bounded near-IK solution '
                f'(position={ik_position_error * 1000.0:.2f} mm, '
                f'rotation={ik_full_rotation_error_deg:.3f} deg)'
            )
        if not self.kinematics.joints_within_limits(solution):
            self._fault('IK solution violates URDF joint limits')
            return None

        preview = JointState()
        preview.header.stamp = self.get_clock().now().to_msg()
        preview.name = list(JOINT_NAMES)
        preview.position = [float(value) for value in solution]
        self.preview_publisher.publish(preview)

        current_tcp = self.kinematics.forward(self.latest_joints)[:3, 3]
        current_error = float(
            np.linalg.norm(desired_position - current_tcp)
        )
        if not math.isfinite(current_error):
            self._fault('forward kinematics returned a non-finite TCP error')
            return None

        command_description = 'NONE (dry-run)'
        if self.execute_hardware:
            command_key = ('CARTESIAN', self.state, self.level)
            if self.command_in_flight != command_key:
                if self.arm_publisher is None:
                    self._fault('arm publisher is unavailable')
                    return None
                arm_message = JointState()
                arm_message.header.stamp = self.get_clock().now().to_msg()
                arm_message.name = list(JOINT_NAMES)
                arm_message.position = [float(value) for value in solution]
                self.arm_publisher.publish(arm_message)
                self.command_in_flight = command_key
                command_description = str(
                    np.round(np.degrees(solution), 2).tolist()
                )
            else:
                command_description = 'WAITING FOR PREVIOUS FULL GOAL'

        self.solution_sequence += 1
        self.get_logger().info(
            f'PLAN {self.solution_sequence} | state={self.state}\n'
            f'  locked_xyz={np.round(self.locked_target, 4).tolist()}\n'
            f'  desired_xyz={np.round(desired_position, 4).tolist()}\n'
            '  solution_deg='
            f'{np.round(np.degrees(solution), 2).tolist()}\n'
            f'  ik_position_error={ik_position_error * 1000.0:.2f} mm\n'
            f'  ik_approach_error={ik_approach_error_deg:.3f} deg\n'
            '  ik_full_orientation_residual='
            f'{ik_full_rotation_error_deg:.3f} deg '
            '(tool-axis roll relaxed)\n'
            f'  current_tcp_error={current_error * 1000.0:.2f} mm\n'
            f'  arm_command_deg={command_description}'
        )
        return (
            current_error,
            float(ik_position_error),
            float(ik_approach_error_deg),
        )

    def send_joint_goal(self, desired_joints, description):
        """Preview a joint goal or approach it with bounded hardware steps."""
        desired = np.asarray(desired_joints, dtype=np.float64)
        if (
            desired.shape != (6,)
            or not np.all(np.isfinite(desired))
            or not self.kinematics.joints_within_limits(desired)
        ):
            self._fault(f'{description} joint goal is invalid')
            return False
        if not self.joints_are_fresh():
            self._fault('joint feedback is stale')
            return False

        preview = JointState()
        preview.header.stamp = self.get_clock().now().to_msg()
        preview.name = list(JOINT_NAMES)
        preview.position = [float(value) for value in desired]
        self.preview_publisher.publish(preview)

        difference = desired - self.latest_joints
        for index in self.kinematics.continuous_joint_indices:
            difference[index] = math.atan2(
                math.sin(difference[index]),
                math.cos(difference[index]),
            )
        if not self.execute_hardware:
            self.get_logger().info(
                f'[DRY-RUN] {description} preview_deg='
                f'{np.round(np.degrees(desired), 2).tolist()}'
            )
            return True
        if float(np.max(np.abs(difference))) <= self.home_tolerance:
            self.command_in_flight = None
            return True
        if self.arm_publisher is None:
            self._fault('arm publisher is unavailable')
            return False

        command_key = ('JOINT', description)
        if self.command_in_flight != command_key:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = [float(value) for value in desired]
            self.arm_publisher.publish(message)
            self.command_in_flight = command_key
        return False

    def _goal_reached(self, result):
        if result is None:
            return False
        current_error, ik_position_error, ik_rotation_error_deg = result
        return goal_reached(
            self.execute_hardware,
            current_error,
            ik_position_error,
            ik_rotation_error_deg,
            self.arrival_tolerance_m,
            self.ik_residual_tolerance_m,
            self.ik_rotation_tolerance_deg,
        )

    def control_loop(self):
        if self.state == 'WAIT_TARGET':
            if self.locked_target is None:
                now = time.monotonic()
                if now - self.last_wait_status_time >= 5.0:
                    self.get_logger().info(
                        'WAITING FOR STABLE ID3 TARGET '
                        f'({len(self.target_samples)}/'
                        f'{self.stable_sample_count} samples)'
                    )
                    self.last_wait_status_time = now
                return
            if not self.joints_are_fresh():
                if (
                    self.locked_target_time is not None
                    and time.monotonic() - self.locked_target_time
                    > self.target_timeout_sec
                ):
                    self._warn_throttled(
                        'Stable target expired while waiting for '
                        'joint feedback'
                    )
                    self._reset_target_sampling()
                return

            if not self._select_square_grasp_orientation():
                return

            if not self.publish_gripper(self.open_pulse, 'OPEN'):
                return
            self.state = 'OPEN'
            self.state_start_time = time.monotonic()
            self.get_logger().info('TARGET LOCKED -> OPEN')
            return

        if self.state == 'OPEN':
            if time.monotonic() - self.state_start_time >= 0.8:
                self.state = 'APPROACH'
                self.level = 0
                self.get_logger().info(
                    'OPEN -> LOCKED-TARGET APPROACH'
                )
            return

        if self.state == 'APPROACH':
            desired = self.locked_target.copy()
            offset = self.z_offsets[self.level]
            desired[2] += offset
            result = self.send_cartesian_goal(desired)
            if not self._goal_reached(result):
                return

            self.get_logger().info(
                f'Approach level {self.level} reached '
                f'(z offset={offset:.3f} m)'
            )
            self.level += 1
            if self.level >= len(self.z_offsets):
                if not self.publish_gripper(self.close_pulse, 'CLOSE'):
                    return
                self.state = 'CLOSE'
                self.state_start_time = time.monotonic()
                self.get_logger().info(
                    'LOCKED GRASP POSITION -> CLOSE'
                )
            return

        if self.state == 'CLOSE':
            if time.monotonic() - self.state_start_time >= 1.0:
                self.state = 'LIFT'
                self.get_logger().info('CLOSE -> LIFT')
            return

        if self.state == 'LIFT':
            desired = self.locked_target.copy()
            desired[2] += self.z_offsets[0]
            result = self.send_cartesian_goal(desired)
            if self._goal_reached(result):
                self.state = 'RETURN_HOME'
                self.get_logger().info(
                    'LIFT -> RETURN_HOME [0, 75, -90, 0, 0, 0] deg'
                )
            return

        if self.state == 'RETURN_HOME':
            if self.send_joint_goal(self.home_joints, 'RETURN_HOME'):
                self.state = 'DONE'
                self.get_logger().info(
                    'VISUAL GRASP PLAN COMPLETE'
                    if not self.execute_hardware
                    else 'VISUAL GRASP HARDWARE SEQUENCE COMPLETE'
                )
            return

        # DONE and FAULT never publish more commands.


def main(args=None):
    rclpy.init(args=args)
    node = VisualGraspController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
