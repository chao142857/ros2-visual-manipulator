"""MoveIt service adapter. Intentionally has no execution action/client.

All requests use the private /tabletop MoveIt instance, complete start states,
the frozen robot model and a solid table/target plus attached open-jaw geometry.
"""

from copy import deepcopy

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
    PlanningScene, RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionFK,
    GetPositionIK, GetStateValidity,
)
from shape_msgs.msg import SolidPrimitive

from .geometry import matrix_from_pose, pose_from_matrix
from .grasps import collision_boxes_in_tcp


ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))
PASSIVE_JOINT = 'tcp_extension_joint'


def robot_state(joints):
    state = RobotState(joint_state=deepcopy(joints), is_diff=False)
    return state


def state_at_end(state, trajectory):
    validate_trajectory(trajectory)
    result = deepcopy(state)
    positions = dict(zip(result.joint_state.name, result.joint_state.position))
    positions.update(zip(trajectory.joint_trajectory.joint_names,
                         trajectory.joint_trajectory.points[-1].positions))
    result.joint_state.position = [float(positions[name]) for name in result.joint_state.name]
    result.joint_state.velocity = []
    result.joint_state.effort = []
    return result


def joint_cost(first, second):
    a = dict(zip(first.joint_state.name, first.joint_state.position))
    b = dict(zip(second.joint_state.name, second.joint_state.position))
    delta = np.array([b[name] - a[name] for name in ARM_JOINTS])
    delta[-1] = np.arctan2(np.sin(delta[-1]), np.cos(delta[-1]))
    return float(np.linalg.norm(delta))


def validate_trajectory(trajectory, max_joint_step=None):
    joint = trajectory.joint_trajectory
    if set(joint.joint_names) != set(ARM_JOINTS) or len(joint.joint_names) != 6 or not joint.points:
        raise ValueError('MoveIt returned an empty or unexpected joint trajectory')
    values = np.array([point.positions for point in joint.points])
    if values.shape != (len(joint.points), 6) or not np.isfinite(values).all():
        raise ValueError('MoveIt returned invalid trajectory positions')
    if max_joint_step is not None and len(values) > 1:
        delta = np.diff(values, axis=0)
        j6 = joint.joint_names.index('joint6')
        delta[:, j6] = np.arctan2(np.sin(delta[:, j6]), np.cos(delta[:, j6]))
        if np.max(np.abs(delta)) > max_joint_step:
            raise ValueError('Cartesian path contains a joint discontinuity')


class MoveItAdapter:
    def __init__(self, node, config):
        self.node, self.cfg = node, config
        self.clients = {name: node.create_client(kind, f'/tabletop/{name}') for name, kind in (
            ('apply_planning_scene', ApplyPlanningScene), ('compute_fk', GetPositionFK),
            ('compute_ik', GetPositionIK), ('plan_kinematic_path', GetMotionPlan),
            ('compute_cartesian_path', GetCartesianPath), ('check_state_validity', GetStateValidity))}
        self.ids = set()

    def available(self):
        return all(client.service_is_ready() for client in self.clients.values())

    def fk(self, state):
        request = GetPositionFK.Request(robot_state=state, fk_link_names=['tool0', 'tcp'])
        request.header.frame_id = self.cfg['base_frame']
        return self.clients['compute_fk'].call_async(request)

    @staticmethod
    def tcp_from_tool(response):
        if response.error_code.val != 1 or set(response.fk_link_names) != {'tool0', 'tcp'}:
            raise ValueError('MoveIt FK failed for tool0/tcp')
        poses = dict(zip(response.fk_link_names, response.pose_stamped))
        return np.linalg.inv(matrix_from_pose(poses['tcp'].pose)) @ \
            matrix_from_pose(poses['tool0'].pose)

    def scene(self, observation, state, tcp_from_tool, gripper):
        # Verify the complete static attachment before submitting any scene.
        boxes = collision_boxes_in_tcp(gripper)
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        definitions = [('tabletop_table', observation.table_pose, observation.table_dimensions)]
        definitions.extend((f'tabletop_{obj.id}', obj.pose, obj.dimensions) for obj in observation.objects)
        new_ids = {item[0] for item in definitions}
        for name in self.ids - new_ids:
            scene.world.collision_objects.append(CollisionObject(id=name, operation=CollisionObject.REMOVE))
        for name, pose, dims in definitions:
            obj = CollisionObject(id=name, operation=CollisionObject.ADD)
            obj.header.frame_id = self.cfg['base_frame']
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[dims.x, dims.y, dims.z])]
            obj.primitive_poses = [pose]
            scene.world.collision_objects.append(obj)
        self.ids = new_ids
        attached = AttachedCollisionObject(link_name='tool0', touch_links=['tool0', 'link6_1_1', 'tcp'])
        attached.object.id = 'tabletop_open_gripper'
        attached.object.header.frame_id = 'tool0'
        attached.object.operation = CollisionObject.ADD
        tool_from_tcp = np.linalg.inv(tcp_from_tool)
        for dimensions, tcp_from_box in boxes:
            attached.object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions))
            attached.object.primitive_poses.append(pose_from_matrix(tool_from_tcp @ tcp_from_box))
        scene.robot_state.attached_collision_objects = [attached]
        state.attached_collision_objects = [deepcopy(attached)]
        return self.clients['apply_planning_scene'].call_async(ApplyPlanningScene.Request(scene=scene))

    def validity(self, state):
        return self.clients['check_state_validity'].call_async(
            GetStateValidity.Request(robot_state=state, group_name=self.cfg['group_name']))

    def ik(self, pose, state, tcp_from_tool):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.cfg['group_name']
        ik.robot_state = state
        ik.avoid_collisions = True
        ik.ik_link_name = 'tool0'
        ik.pose_stamped = PoseStamped(pose=pose_from_matrix(pose @ tcp_from_tool))
        ik.pose_stamped.header.frame_id = self.cfg['base_frame']
        duration = self.cfg['ik_timeout_sec']
        ik.timeout.sec, ik.timeout.nanosec = int(duration), int(duration % 1 * 1e9)
        return self.clients['compute_ik'].call_async(request)

    def plan_pregrasp(self, state, goal):
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = self.cfg['group_name']
        plan.pipeline_id = 'ompl'
        plan.planner_id = 'RRTConnectkConfigDefault'
        plan.start_state = state
        plan.allowed_planning_time = self.cfg['planning_time_sec']
        plan.num_planning_attempts = self.cfg['planning_attempts']
        plan.max_velocity_scaling_factor = self.cfg['velocity_scale']
        plan.max_acceleration_scaling_factor = self.cfg['acceleration_scale']
        positions = dict(zip(goal.joint_state.name, goal.joint_state.position))
        tolerance = self.cfg['joint_goal_tolerance_rad']
        plan.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=positions[name], tolerance_above=tolerance,
                            tolerance_below=tolerance, weight=1.0) for name in ARM_JOINTS])]
        return self.clients['plan_kinematic_path'].call_async(request)

    def cartesian(self, state, goal, tcp_from_tool):
        request = GetCartesianPath.Request()
        request.header.frame_id = self.cfg['base_frame']
        request.start_state = state
        request.group_name = self.cfg['group_name']
        request.link_name = 'tool0'
        request.waypoints = [pose_from_matrix(goal @ tcp_from_tool)]
        request.max_step = self.cfg['cartesian_step_m']
        request.jump_threshold = 0.0  # Absolute thresholds below avoid scene-dependent scaling.
        request.revolute_jump_threshold = self.cfg['max_cartesian_joint_step_rad']
        request.prismatic_jump_threshold = self.cfg['max_cartesian_prismatic_step_m']
        request.avoid_collisions = True
        return self.clients['compute_cartesian_path'].call_async(request)
