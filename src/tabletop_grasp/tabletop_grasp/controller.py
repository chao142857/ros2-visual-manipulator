"""Explicit asynchronous perception -> MoveIt plan state machine, with no execution."""

from copy import deepcopy
import json
import time

from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tabletop_grasp_interfaces.msg import TabletopScene
from visualization_msgs.msg import MarkerArray

from .config import GraspConfig, declare_config
from .geometry import matrix_from_pose, pose_from_matrix
from .grasps import generate_grasps
from .joint_feedback import JointFeedback
from .moveit_adapter import (
    PASSIVE_JOINT, MoveItAdapter, joint_cost, robot_state, state_at_end,
    validate_trajectory,
)
from .ros_geometry import Markers, box_from_message, stamped


DEFAULTS = {
    'dry_run': True, 'base_frame': 'base_link', 'group_name': 'arm',
    'joint_states_topic': '/joint_states', 'automatic_planning': True,
    'scene_timeout_sec': 1.5, 'joint_timeout_sec': 1.0, 'future_tolerance_sec': 0.05,
    'stable_observations': 3, 'scene_position_tolerance_m': 0.008,
    'scene_dimension_tolerance_m': 0.008, 'scene_axis_tolerance_deg': 8.0,
    'joint_drift_tolerance_rad': 0.015, 'passive_drift_tolerance_m': 0.001,
    'planning_time_sec': 3.0, 'planning_attempts': 2, 'ik_timeout_sec': 0.15,
    'service_timeout_sec': 8.0, 'retry_period_sec': 3.0,
    'velocity_scale': 0.1, 'acceleration_scale': 0.1, 'joint_goal_tolerance_rad': 0.002,
    'cartesian_step_m': 0.004, 'max_cartesian_joint_step_rad': 0.35,
    'max_cartesian_prismatic_step_m': 0.003,
    'marker_lifetime_sec': 1.5,
}


def validate_planning_domain(gripper, domain_id):
    if gripper.geometry_source == 'synthetic_fixture' and domain_id != 191:
        raise ValueError('synthetic_fixture geometry requires isolated ROS_DOMAIN_ID=191')


def equivalent_scene(first, second, config):
    """Compare complete scenes, treating a cuboid yaw plus pi as equivalent."""
    if len(first.objects) != len(second.objects) or first.target_index != second.target_index:
        return False
    pa, pb = matrix_from_pose(first.table_pose), matrix_from_pose(second.table_pose)
    angle = np.cos(np.radians(config['scene_axis_tolerance_deg']))
    if np.linalg.norm(pa[:3, 3] - pb[:3, 3]) > config['scene_position_tolerance_m'] or \
            np.dot(pa[:3, 2], pb[:3, 2]) < angle:
        return False
    # Table footprint can grow/shrink with occlusion: do not retain a plan through that change.
    da, db = first.table_dimensions, second.table_dimensions
    if np.max(np.abs(np.array([da.x, da.y, da.z]) - [db.x, db.y, db.z])) > \
            config['scene_dimension_tolerance_m']:
        return False
    for a, b in zip(first.objects, second.objects):
        a, b = box_from_message(a), box_from_message(b)
        if (np.linalg.norm(a.center - b.center) > config['scene_position_tolerance_m']
                or np.max(np.abs(a.dimensions - b.dimensions)) > config['scene_dimension_tolerance_m']
                or abs(np.dot(a.rotation[:, 0], b.rotation[:, 0])) < angle
                or np.dot(a.rotation[:, 2], b.rotation[:, 2]) < angle
                or a.yaw_observable != b.yaw_observable):
            return False
    return True


class PlannerNode(Node):
    def __init__(self):
        super().__init__('tabletop_planner')
        self.gripper = declare_config(self, GraspConfig, 'gripper')
        validate_planning_domain(self.gripper, self.context.get_domain_id())
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in DEFAULTS}
        if self.cfg['dry_run'] is not True:
            raise ValueError('dry_run=false is unsupported: this package has no execution adapter')
        for name, value in self.cfg.items():
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                if not np.isfinite(value) or value <= 0:
                    raise ValueError(f'{name} must be finite and positive')
        if max(self.cfg['velocity_scale'], self.cfg['acceleration_scale']) > 1:
            raise ValueError('planning speed scales must not exceed 1')
        self.adapter = MoveItAdapter(self, self.cfg)
        self.feedback = JointFeedback(self.cfg['joint_timeout_sec'], self.cfg['future_tolerance_sec'])
        self.state = 'IDLE'
        self.reason = ''
        self.scene = self.locked = self.joints = self.start = None
        self.pending = None
        self.stable_count = 0
        self.not_before = 0.0
        self.requested = self.cfg['automatic_planning']
        self.candidates, self.feasible, self.trajectories, self.rejections = [], [], [], []
        self.selected = None
        self.status_pub = self.create_publisher(String, '/tabletop/planner_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/tabletop/grasp_markers', 1)
        self.display_pub = self.create_publisher(DisplayTrajectory, '/tabletop/display_planned_path', 1)
        self.candidate_pub = self.create_publisher(PoseArray, '/tabletop/grasp_candidates', 1)
        self.pose_pubs = {name: self.create_publisher(PoseStamped, f'/tabletop/{name}', 1)
                          for name in ('selected_grasp', 'pregrasp', 'grasp', 'retreat')}
        self.create_subscription(TabletopScene, '/tabletop/scene', self._scene, 1)
        self.create_subscription(JointState, self.cfg['joint_states_topic'], self._joints,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/tabletop/replan', self._replan)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._publish_status)
        self._transition('PERCEIVE', 'planning only; execution interface absent')

    def _fresh(self, header, timeout):
        ns = Time.from_msg(header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - ns) / 1e9
        return ns > 0 and -self.cfg['future_tolerance_sec'] <= age <= timeout

    def _scene(self, scene):
        try:
            if (not scene.valid or scene.header.frame_id != self.cfg['base_frame']
                    or not self._fresh(scene.header, self.cfg['scene_timeout_sec'])
                    or not 0 <= scene.target_index < len(scene.objects)):
                raise ValueError(scene.reason or 'invalid/stale scene or wrong planning frame')
            matrix_from_pose(scene.table_pose)
            dims = scene.table_dimensions
            if not np.isfinite([dims.x, dims.y, dims.z]).all() or min(dims.x, dims.y, dims.z) <= 0:
                raise ValueError('invalid table collision dimensions')
            if self.scene is not None and Time.from_msg(scene.header.stamp) <= \
                    Time.from_msg(self.scene.header.stamp):
                return  # Repeated/out-of-order frames cannot satisfy the stability gate.
            for obj in scene.objects:
                box_from_message(obj)
            if self.locked is not None and not equivalent_scene(self.locked, scene, self.cfg):
                self._abort('scene changed during planning/preview')
            self.stable_count = self.stable_count + 1 if self.scene is not None and \
                equivalent_scene(self.scene, scene, self.cfg) else 1
            self.scene = scene
        except (ValueError, RuntimeError) as error:
            self.scene = None
            self.stable_count = 0
            self._abort(str(error))

    def _joints(self, msg):
        try:
            msg = self.feedback.update(msg, self.get_clock().now().nanoseconds)
        except ValueError:
            msg = None
        if msg is None:
            self.joints = None
            if self.locked is not None:
                self._abort('invalid/stale joints or missing passive TCP state')
            return
        self.joints = msg
        if self.start is not None:
            if joint_cost(self.start, robot_state(msg)) > self.cfg['joint_drift_tolerance_rad']:
                self._abort('robot joint state changed during planning/preview')
                return
            a = dict(zip(self.start.joint_state.name, self.start.joint_state.position))[PASSIVE_JOINT]
            b = dict(zip(msg.name, msg.position))[PASSIVE_JOINT]
            if abs(a - b) > self.cfg['passive_drift_tolerance_m']:
                self._abort('passive TCP extension changed during planning/preview')

    def _replan(self, request, response):
        self._abort('new planning request')
        self.requested = True
        self.not_before = 0.0
        response.success = True
        response.message = 'Fresh stable geometry and joint feedback required; planning only'
        return response

    def _transition(self, state, reason=''):
        if (state, reason) != (self.state, self.reason):
            self.get_logger().info(f'{self.state} -> {state}: {reason}')
        self.state, self.reason = state, reason
        self._publish_status()

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': self.reason, 'dry_run': True,
            'selected': self.selected.id if self.selected else None,
            'rejections': self.rejections,
            'geometry_verified': not self.gripper.planning_rejections(),
            'geometry_source': self.gripper.geometry_source,
            'verification': self.gripper.verification(),
            'planning_blockers': self.gripper.planning_rejections(),
        })))
        if self.candidates:
            self._visualize()  # Only retained while the observation/freshness gates pass.

    def _abort(self, reason):
        if self.pending is not None:
            self.pending[1].cancel()
        self.pending = self.locked = self.start = self.selected = None
        self.candidates, self.feasible, self.trajectories = [], [], []
        self.stable_count = 0
        self.not_before = time.monotonic() + self.cfg['retry_period_sec']
        self.marker_pub.publish(Markers(self._header(), 'grasp').array)
        self.candidate_pub.publish(PoseArray(header=self._header()))
        self.display_pub.publish(DisplayTrajectory())
        self._transition('ABORT', reason)

    def _header(self):
        return Header(frame_id=self.cfg['base_frame'], stamp=self.get_clock().now().to_msg())

    def _call(self, kind, future):
        self.pending = (kind, future, time.monotonic())

    def _tick(self):
        try:
            if self.locked is not None:
                if self.gripper.planning_rejections():
                    self._abort('; '.join(self.gripper.planning_rejections()))
                    return
                if self.scene is None or not self._fresh(self.scene.header, self.cfg['scene_timeout_sec']):
                    self._abort('scene expired; discarded selected grasp and trajectories')
                    return
                if self.joints is None or not self._fresh(self.joints.header, self.cfg['joint_timeout_sec']):
                    self._abort('joint feedback expired')
                    return
            if self.pending is not None:
                kind, future, since = self.pending
                if time.monotonic() - since > self.cfg['service_timeout_sec']:
                    self._abort(f'MoveIt {kind} service timeout')
                elif future.done():
                    self.pending = None
                    self._response(kind, future.result())
                return
            if self.state in ('IDLE', 'PERCEIVE', 'ABORT') and self.requested:
                if (time.monotonic() < self.not_before or self.scene is None
                        or self.stable_count < self.cfg['stable_observations']):
                    return
                if not self._fresh(self.scene.header, self.cfg['scene_timeout_sec']):
                    self._abort('scene expired before planning')
                    return
                self._transition('TARGET_VALIDATION')
                self.candidates, self.rejections = generate_grasps(
                    box_from_message(self.scene.objects[self.scene.target_index]), self.gripper)
                if not self.candidates:
                    self._abort('; '.join(self.rejections))
                    return
                self._transition('GENERATE_GRASPS', f'{len(self.candidates)} short-edge candidates')
                self._visualize()
                if self.joints is None or not self._fresh(self.joints.header, self.cfg['joint_timeout_sec']):
                    self._abort('fresh six-joint feedback and passive TCP extension required')
                    return
                if not self.adapter.available():
                    self._abort('private MoveIt planning services unavailable')
                    return
                self.locked = deepcopy(self.scene)
                self.start = robot_state(self.joints)
                self._call('fk', self.adapter.fk(self.start))
        except Exception as error:
            self._abort(f'{type(error).__name__}: {error}')

    def _response(self, kind, response):
        if response is None:
            raise ValueError(f'empty MoveIt {kind} response')
        if kind == 'fk':
            self.tcp_from_tool = self.adapter.tcp_from_tool(response)
            self._call('scene', self.adapter.scene(self.locked, self.start, self.tcp_from_tool, self.gripper))
        elif kind == 'scene':
            if not response.success:
                raise ValueError('MoveIt refused planning scene')
            self._call('validity', self.adapter.validity(self.start))
        elif kind == 'validity':
            if not response.valid:
                contacts = [(c.contact_body_1, c.contact_body_2) for c in response.contacts]
                raise ValueError(f'start state in collision or outside constraints: {contacts}')
            self.ik_index = 0
            self.feasible = []
            self._next_ik()
        elif kind == 'ik_pre':
            if response.error_code.val != 1:
                self.rejections.append(f'{self.candidates[self.ik_index].id}: pregrasp IK '
                                       f'{response.error_code.val}')
                self.ik_index += 1
                self._next_ik()
            else:
                self.pre_solution = response.solution
                self._call('ik_grasp', self.adapter.ik(
                    self.candidates[self.ik_index].grasp, self.pre_solution, self.tcp_from_tool))
        elif kind == 'ik_grasp':
            if response.error_code.val == 1:
                candidate = self.candidates[self.ik_index]
                self.feasible.append((joint_cost(self.start, self.pre_solution)
                                      + joint_cost(self.pre_solution, response.solution),
                                      candidate, self.pre_solution))
            else:
                self.rejections.append(f'{self.candidates[self.ik_index].id}: grasp IK '
                                       f'{response.error_code.val}')
            self.ik_index += 1
            self._next_ik()
        elif kind == 'pregrasp':
            plan = response.motion_plan_response
            if plan.error_code.val != 1:
                self._reject_plan(f'pregrasp path {plan.error_code.val}')
                return
            validate_trajectory(plan.trajectory)
            self.trajectories = [plan.trajectory]
            self.stage_start = state_at_end(self.start, plan.trajectory)
            self._transition('PLAN_APPROACH')
            self._call('approach', self.adapter.cartesian(
                self.stage_start, self.trying.grasp, self.tcp_from_tool))
        elif kind in ('approach', 'retreat'):
            if response.error_code.val != 1 or not np.isfinite(response.fraction) or \
                    response.fraction < 1.0 - 1e-6:
                self._reject_plan(f'{kind} incomplete: fraction={response.fraction:.4f}, '
                                  f'code={response.error_code.val}')
                return
            try:
                validate_trajectory(response.solution, self.cfg['max_cartesian_joint_step_rad'])
            except ValueError as error:
                self._reject_plan(str(error))
                return
            self.trajectories.append(response.solution)
            self.stage_start = state_at_end(self.stage_start, response.solution)
            if kind == 'approach':
                self._transition('PLAN_RETREAT', 'open-gripper withdrawal; no close/attachment step')
                self._call('retreat', self.adapter.cartesian(
                    self.stage_start, self.trying.retreat, self.tcp_from_tool))
            else:
                self.selected = self.trying
                self._transition('READY_TO_EXECUTE', 'DRY RUN ONLY: all three paths collision-checked')
                self.display_pub.publish(DisplayTrajectory(
                    trajectory_start=self.start, trajectory=self.trajectories))
                self._visualize()
                if not self.cfg['automatic_planning']:
                    self.requested = False

    def _next_ik(self):
        self._transition('SELECT_GRASP', 'collision-aware KDL IK and wrapped J6 distance ranking')
        if self.ik_index < len(self.candidates):
            self._call('ik_pre', self.adapter.ik(
                self.candidates[self.ik_index].pregrasp, self.start, self.tcp_from_tool))
        else:
            self.feasible.sort(key=lambda item: (item[0], item[1].id))
            self._next_plan()

    def _next_plan(self):
        if not self.feasible:
            self._abort('no collision/reachability/path-valid grasp; see rejections')
            return
        _, self.trying, goal = self.feasible.pop(0)
        self.trajectories = []
        self._transition('PLAN_PREGRASP', self.trying.id)
        self._call('pregrasp', self.adapter.plan_pregrasp(self.start, goal))

    def _reject_plan(self, reason):
        self.rejections.append(f'{self.trying.id}: {reason}')
        self.get_logger().warning(self.rejections[-1])
        self._next_plan()

    def _visualize(self):
        header = self._header()
        markers = Markers(header, 'grasps', self.cfg['marker_lifetime_sec'])
        self.candidate_pub.publish(PoseArray(
            header=header, poses=[pose_from_matrix(c.grasp) for c in self.candidates]))
        for candidate in self.candidates:
            markers.arrow(candidate.pregrasp[:3, 3], candidate.grasp[:3, 3], (1, 0.6, 0, 0.65))
            closing = candidate.grasp[:3, :3] @ np.array([1., 0., 0.])
            markers.arrow(candidate.grasp[:3, 3] - closing * candidate.required_opening_m / 2,
                          candidate.grasp[:3, 3] + closing * candidate.required_opening_m / 2,
                          (1, 0.6, 0, 0.9))
        if self.selected:
            c = self.selected
            for name, pose in (('selected_grasp', c.grasp), ('pregrasp', c.pregrasp),
                               ('grasp', c.grasp), ('retreat', c.retreat)):
                self.pose_pubs[name].publish(stamped(header, pose))
            markers.axes(c.grasp[:3, 3], c.grasp[:3, :3])
            markers.arrow(c.pregrasp[:3, 3], c.grasp[:3, 3], (0.1, 1, 0.4, 1), 0.007)
            markers.arrow(c.grasp[:3, 3], c.retreat[:3, 3], (0.2, 0.7, 1, 1))
            label = c.retreat[:3, 3] + np.array([0., 0., 0.035])
            markers.label(label, 'DRY RUN ONLY\n' + c.id)
        self.marker_pub.publish(markers.array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
