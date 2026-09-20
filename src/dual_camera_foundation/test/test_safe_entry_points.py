from pathlib import Path
import re

import pytest

from dual_camera_foundation.visual_grasp_safety import (
    validate_gripper_configuration,
)


PACKAGE = Path(__file__).parents[1]


def test_canonical_system_requires_explicit_actuation_authority():
    text = (PACKAGE / 'launch' / 'system_bringup.launch.py').read_text()
    assert re.search(
        r"'arm_allow_motion_commands',\s*default_value='false'",
        text,
    )
    assert re.search(
        r"'gripper_allow_hardware_actuation',\s*default_value='false'", text
    )
    assert 'gripper_pulses_confirmed' not in text


def test_feedback_stage_keeps_gripper_driver_stopped():
    text = (PACKAGE / 'launch' / 'arm_feedback_safe.launch.py').read_text()
    assert "'arm_allow_motion_commands': 'false'" in text
    assert "'start_gripper_driver': 'false'" in text
    assert "DeclareLaunchArgument('arm_allow_motion_commands'" not in text
    assert "DeclareLaunchArgument('gripper_allow_commands'" not in text
    assert re.search(
        r"'arm_use_mock',\s*default_value='true'",
        text,
    )


def test_experimental_gripper_uses_current_physical_pulses():
    controller = (
        PACKAGE
        / 'dual_camera_foundation'
        / 'visual_grasp_controller.py'
    ).read_text()
    assert "declare_parameter('gripper_open_pulse_us', 600)" in controller
    assert "declare_parameter('gripper_close_pulse_us', 1300)" in controller
    assert 'self.close_pulse = 1550' not in controller
    assert 'HARDWARE_EXECUTION_UNLOCKED = False' in controller


def test_hardware_gripper_requires_only_in_range_pulses():
    assert validate_gripper_configuration(
        900, 1400
    ) == (900, 1400)
    with pytest.raises(ValueError, match='between 600 and 1700'):
        validate_gripper_configuration(599, 1300)
