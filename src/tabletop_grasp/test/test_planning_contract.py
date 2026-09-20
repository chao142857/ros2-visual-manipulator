from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Vector3
from moveit_msgs.msg import RobotState, RobotTrajectory
import numpy as np
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tabletop_grasp_interfaces.msg import OrientedObject, TabletopScene
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tabletop_grasp.config import GraspConfig
from tabletop_grasp.controller import DEFAULTS, PlannerNode, equivalent_scene
from tabletop_grasp.geometry import pose_from_matrix
from tabletop_grasp.moveit_adapter import (
    ARM_JOINTS, MoveItAdapter, joint_cost, robot_state, state_at_end, validate_trajectory,
)


def test_real_pipeline_scene_and_marker_ros_serialization():
    from rclpy.serialization import serialize_message
    from tabletop_grasp.config import PerceptionConfig
    from tabletop_grasp.pipeline import TabletopPipeline
    from tabletop_grasp.ros_geometry import observation_markers, scene_message
    from tabletop_grasp.synthetic_scene import cuboid_scene
    result = TabletopPipeline(PerceptionConfig()).process(cuboid_scene())
    assert result.valid
    assert serialize_message(scene_message(Header(frame_id='base_link'), result))
    assert serialize_message(observation_markers(Header(frame_id='base_link'), result, 1.5))


def scene():
    matrix = np.eye(4)
    matrix[:3, 3] = [0.03, -0.28, 0.01]
    obj = OrientedObject(id='object_0', pose=pose_from_matrix(matrix),
                         dimensions=Vector3(x=0.08, y=0.034, z=0.04), yaw_observable=True)
    return TabletopScene(valid=True, target_index=0, objects=[obj],
                         table_pose=pose_from_matrix(np.eye(4)),
                         table_dimensions=Vector3(x=0.4, y=0.4, z=0.03))


def test_scene_matching_allows_pi_but_rejects_target_or_table_motion():
    first, second = scene(), scene()
    second.objects[0].pose.orientation.z = 1.
    second.objects[0].pose.orientation.w = 0.
    assert equivalent_scene(first, second, DEFAULTS)
    second.objects[0].pose.position.x += 0.02
    assert not equivalent_scene(first, second, DEFAULTS)
    second = scene()
    second.table_pose.position.y += 0.02
    assert not equivalent_scene(first, second, DEFAULTS)


def test_j6_cost_wraps_without_wrapping_bounded_joints():
    first = RobotState(joint_state=JointState(name=list(ARM_JOINTS), position=[0.] * 5 + [3.13]))
    second = RobotState(joint_state=JointState(name=list(ARM_JOINTS), position=[0.] * 5 + [-3.13]))
    assert joint_cost(first, second) < 0.03
    second.joint_state.position[0] = 3.13
    assert joint_cost(first, second) > 3.


def trajectory(values):
    return RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=list(ARM_JOINTS),
                           points=[JointTrajectoryPoint(positions=list(row)) for row in values]))


@pytest.mark.parametrize('values', [[], [[0.] * 5 + [np.nan]], [[0.] * 6, [1.] * 6]])
def test_empty_nonfinite_and_discontinuous_paths_fail(values):
    with pytest.raises(ValueError):
        validate_trajectory(trajectory(values), 0.35)


def test_segment_chaining_preserves_passive_tcp():
    state = robot_state(JointState(
        name=list(ARM_JOINTS) + ['tcp_extension_joint'], position=[0.] * 6 + [0.026]))
    result = state_at_end(state, trajectory([[0.1] * 6]))
    assert list(result.joint_state.position) == [0.1] * 6 + [0.026]
    assert list(state.joint_state.position) == [0.] * 6 + [0.026]


class Client:
    def call_async(self, request):
        self.request = request
        return request

    def service_is_ready(self):
        return True


class FakeNode:
    def __init__(self):
        self.clients = {}

    def create_client(self, kind, topic):
        self.clients[topic] = Client()
        return self.clients[topic]


def test_moveit_requests_include_collisions_and_no_execution_interface():
    node = FakeNode()
    adapter = MoveItAdapter(node, DEFAULTS)
    start = RobotState()
    adapter.ik(np.eye(4), start, np.eye(4))
    assert node.clients['/tabletop/compute_ik'].request.ik_request.avoid_collisions
    adapter.cartesian(start, np.eye(4), np.eye(4))
    assert node.clients['/tabletop/compute_cartesian_path'].request.avoid_collisions
    assert all('execute' not in name and 'arm/' not in name for name in node.clients)
    config = GraspConfig(geometry_source='synthetic_fixture', cad_geometry_verified=True,
                         grasp_dimensions_verified=True, jaw_opening_range_verified=True,
                         jaw_configuration_verified=True, tcp_verified=True,
                         cad_attachment_verified=True, camera_extrinsics_verified=True,
                         current_opening_m=0.075, max_opening_m=0.075,
                         finger_thickness_m=0.006, finger_width_m=0.012, finger_length_m=0.035,
                         fingertip_below_tcp_m=0.005, palm_depth_m=0.015, palm_width_m=0.025)
    tcp_tool = np.eye(4)
    tcp_tool[1, 3] = 0.1
    adapter.scene(scene(), start, tcp_tool, config)
    result = node.clients['/tabletop/apply_planning_scene'].request.scene
    assert len(result.world.collision_objects) == 2
    assert result.world.collision_objects[0].id == 'tabletop_table'
    assert len(start.attached_collision_objects[0].object.primitives) == 3
    assert result.allowed_collision_matrix.entry_names == []


def test_abort_cancels_future_and_clears_all_cached_planning_outputs():
    # Exercise the real abort method without constructing a DDS/hardware node.
    output = []
    pending = SimpleNamespace(cancel=lambda: output.append('cancelled'))
    node = SimpleNamespace(pending=('ik', pending, 0.), locked=scene(), start=RobotState(),
                           selected=object(), candidates=[object()], feasible=[object()],
                           trajectories=[object()], stable_count=5, cfg=DEFAULTS,
                           marker_pub=SimpleNamespace(publish=output.append),
                           candidate_pub=SimpleNamespace(publish=output.append),
                           display_pub=SimpleNamespace(publish=output.append),
                           _header=lambda: Header(), _transition=lambda *args: output.append(args))
    PlannerNode._abort(node, 'expired')
    assert node.pending is node.locked is node.start is node.selected is None
    assert node.trajectories == node.candidates == node.feasible == []
    assert output[0] == 'cancelled'
    assert output[-1] == ('ABORT', 'expired')


def test_launch_and_runtime_have_no_hardware_or_execute_clients():
    package = Path(__file__).parents[1]
    runtime = '\n'.join(path.read_text() for path in (package / 'tabletop_grasp').glob('*.py'))
    assert 'from serial' not in runtime and 'import serial' not in runtime
    assert 'ActionClient(' not in runtime
    for name in ('/arm/joint_command', '/gripper/pulse_us', '/arm/stop'):
        assert name not in runtime
    launch = (package / 'launch/moveit_planning.launch.py').read_text()
    assert "'allow_trajectory_execution': False" in launch
    assert "'moveit_manage_controllers': False" in launch
    assert 'move_group/MoveGroupExecuteTrajectoryAction' in launch
    assert 'move_group/MoveGroupMoveAction' in launch
    assert not (package / 'config/joint_limits.yaml').exists()
    model_path = package / 'config/dummy_v2.srdf'
    if not model_path.exists() and (package.parents[1] / 'PUBLICATION_MANIFEST.json').is_file():
        pytest.skip('Software safety assertions completed; public release excludes model SRDF')
    model = model_path.read_text()
    assert 'passive_joint name="tcp_extension_joint"' in model
    assert 'reason="Never"' not in model
