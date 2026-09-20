from pathlib import Path

import numpy as np

from dual_camera_foundation.visual_grasp_controller import (
    VisualGraspController,
)
from dual_camera_foundation.visual_grasp_safety import (
    goal_reached,
    point_in_workspace,
    stable_median_target,
    target_sample_span,
)


LOWER = np.array([-0.55, -0.55, 0.0])
UPPER = np.array([0.55, 0.55, 0.65])


def test_stable_points_lock_to_coordinate_median():
    samples = [
        [0.30, -0.20, 0.10],
        [0.302, -0.201, 0.101],
        [0.299, -0.198, 0.099],
        [0.301, -0.202, 0.102],
        [0.298, -0.199, 0.100],
        [0.300, -0.200, 0.101],
        [0.301, -0.201, 0.100],
    ]
    target, span = stable_median_target(
        samples,
        7,
        0.015,
        LOWER,
        UPPER,
    )
    assert target is not None
    assert span < 0.015
    assert np.allclose(target, [0.300, -0.200, 0.100])


def test_wide_or_nonfinite_samples_never_lock():
    wide = [[0.20, 0.0, 0.1]] * 6 + [[0.23, 0.0, 0.1]]
    target, span = stable_median_target(
        wide,
        7,
        0.015,
        LOWER,
        UPPER,
    )
    assert target is None
    assert span > 0.015
    assert target_sample_span([[0.0, float('nan'), 0.0]]) == float('inf')


def test_workspace_is_closed_and_rejects_bad_shapes():
    assert point_in_workspace(LOWER, LOWER, UPPER)
    assert point_in_workspace(UPPER, LOWER, UPPER)
    assert not point_in_workspace([0.56, 0.0, 0.1], LOWER, UPPER)
    assert not point_in_workspace([0.0, 0.0], LOWER, UPPER)


def test_preview_progress_uses_ik_residual_not_current_tcp_error():
    assert goal_reached(False, 0.30, 0.001, 0.5, 0.018, 0.003, 2.0)
    assert not goal_reached(False, 0.0, 0.004, 0.5, 0.018, 0.003, 2.0)
    assert not goal_reached(False, 0.0, 0.001, 3.0, 0.018, 0.003, 2.0)
    assert not goal_reached(True, 0.30, 0.001, 0.5, 0.018, 0.003, 2.0)


def test_near_ik_solution_is_accepted_only_inside_configured_bounds():
    controller = VisualGraspController.__new__(VisualGraspController)
    controller.ik_residual_tolerance_m = 0.005
    controller.ik_rotation_tolerance_deg = 2.0
    solution = np.zeros(6)

    assert controller._ik_result_is_usable(
        False, solution, 0.0037, 0.9, 0.9
    )
    assert not controller._ik_result_is_usable(
        False, solution, 0.0051, 0.9, 0.9
    )
    assert not controller._ik_result_is_usable(
        False, solution, 0.0037, 2.1, 0.9
    )


def test_controller_source_keeps_hardware_publishers_behind_opt_in():
    controller = (
        Path(__file__).parents[1]
        / 'dual_camera_foundation'
        / 'visual_grasp_controller.py'
    ).read_text()
    assert "declare_parameter('execute_hardware', False)" in controller
    gate = controller.index('if self.execute_hardware:')
    assert controller.index("'/arm/joint_command'") > gate
    assert controller.index("'/gripper/pulse_us'") > gate
    assert 'HARDWARE_EXECUTION_UNLOCKED = False' in controller


def test_controller_sends_full_goals_once_without_degree_step_clipping():
    controller = (
        Path(__file__).parents[1]
        / 'dual_camera_foundation'
        / 'visual_grasp_controller.py'
    ).read_text()
    assert 'max_joint_step' not in controller
    assert "command_key = ('CARTESIAN', self.state, self.level)" in controller
    assert "command_key = ('JOINT', description)" in controller
    assert 'position = [float(value) for value in solution]' in controller
    assert 'position = [float(value) for value in desired]' in controller


def test_controller_uses_reachable_staged_clearance_for_lift_too():
    controller = (
        Path(__file__).parents[1]
        / 'dual_camera_foundation'
        / 'visual_grasp_controller.py'
    ).read_text()
    assert 'self.z_offsets = [0.04, 0.02, 0.00]' in controller
    assert 'desired[2] += self.z_offsets[0]' in controller


def test_d405_id3_launch_redirects_to_tag_free_planning_only():
    launch = (
        Path(__file__).parents[1]
        / 'launch'
        / 'd405_id3_grasp.launch.py'
    ).read_text()
    assert "FindPackageShare('tabletop_grasp')" in launch
    assert 'tabletop_grasp.launch.py' in launch
    assert 'grasp_offset_m' not in launch
    assert 'start_arm_driver' not in launch
    assert 'start_gripper_driver' not in launch
