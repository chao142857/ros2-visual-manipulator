import importlib.util
from pathlib import Path
import re

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest


PACKAGE = Path(__file__).parents[1]


def test_console_launch_forwards_disabled_arm_authority_default():
    text = (PACKAGE / 'launch' / 'arm_console.launch.py').read_text()
    assert re.search(
        r"'arm_allow_motion_commands',\s*default_value='false'",
        text,
    )
    assert "'allow_motion_commands': ParameterValue(" in text


def test_console_and_astra_wrapper_require_gripper_hardware_opt_in():
    console = (PACKAGE / 'launch' / 'arm_console.launch.py').read_text()
    astra = (PACKAGE / 'launch' / 'arm_console_astra.launch.py').read_text()
    for text in (console, astra):
        assert re.search(
            r"'gripper_allow_hardware_actuation',\s*default_value='false'",
            text,
        )
        assert 'gripper_pulses_confirmed' not in text
    assert re.search(
        r"'gripper_open_pulse_us',\s*default_value='600'",
        astra,
    )
    assert re.search(
        r"'gripper_close_pulse_us',\s*default_value='1300'",
        astra,
    )
    assert "'gripper_open_pulse_us': gripper_open_pulse_us" in astra
    assert "'gripper_close_pulse_us': gripper_close_pulse_us" in astra


def test_console_source_has_no_gripper_authority_gates():
    source = (PACKAGE / 'arm_console' / 'ros_bridge.py').read_text()
    assert 'gripper_allow_commands' not in source
    assert 'gripper_pulses_confirmed' not in source


@pytest.fixture
def inspect_launch(monkeypatch):
    """Construct launch actions while forbidding every ROS Node execution."""
    monkeypatch.syspath_prepend(str(PACKAGE.parent / 'dual_camera_foundation'))

    class InspectableNode(Node):
        def __init__(self, **kwargs):
            self.audit_parameters = kwargs.get('parameters', [])
            super().__init__(**kwargs)

        def execute(self, _context):
            pytest.fail('launch safety tests must never execute a ROS node')

    def load(package, filename, overrides=None):
        path = PACKAGE.parent / package / 'launch' / filename
        spec = importlib.util.spec_from_file_location('safety_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        if hasattr(module, 'Node'):
            monkeypatch.setattr(module, 'Node', InspectableNode)
        if hasattr(module, 'get_package_share_directory'):
            monkeypatch.setattr(
                module, 'get_package_share_directory',
                lambda name: str(PACKAGE.parent / name),
            )
        description = module.generate_launch_description()
        context = LaunchContext()
        # Only resolve source paths; no installed overlay or model is required.
        context.launch_configurations.update({
            'arm_config_file': str(
                PACKAGE.parent / 'arm_console/config/arm_console.yaml'
            ),
            'extrinsics_file': str(
                PACKAGE.parent / 'camera_perception'
                / 'config/camera_extrinsics.yaml'
            ),
        })
        context.launch_configurations.update(overrides or {})
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return module, description, context

    return load


def _include_arguments(action, context):
    def resolve(value):
        return perform_substitutions(
            context, normalize_to_list_of_substitutions(value),
        )
    return {resolve(key): resolve(value)
            for key, value in action.launch_arguments}


def _driver_parameters(actions, context):
    result = {}
    for action in actions:
        if not isinstance(action, Node):
            continue
        if action.node_package not in ('arm_driver', 'gripper_driver'):
            continue
        if (
            action.condition is not None
            and not action.condition.evaluate(context)
        ):
            continue
        dictionaries = [item for item in action.audit_parameters
                        if isinstance(item, dict)]
        values = {}
        evaluated = evaluate_parameters(
            context, normalize_parameters(dictionaries),
        )
        for item in evaluated:
            values.update(item)
        result[action.node_package] = values
    return result


@pytest.mark.parametrize('hardware_opt_in', [False, True])
def test_console_resolves_typed_driver_safety_parameters(
    inspect_launch, hardware_opt_in,
):
    overrides = {'start_driver': 'true', 'start_gripper_driver': 'true'}
    if hardware_opt_in:
        overrides.update(use_mock='false', arm_allow_motion_commands='true',
                         gripper_allow_hardware_actuation='true')
    _, description, context = inspect_launch(
        'arm_console', 'arm_console.launch.py', overrides,
    )
    drivers = _driver_parameters(description.entities, context)
    assert drivers['arm_driver']['use_mock'] is not hardware_opt_in
    assert drivers['arm_driver']['allow_motion_commands'] is hardware_opt_in
    assert (
        drivers['gripper_driver']['allow_hardware_actuation']
        is hardware_opt_in
    )


@pytest.mark.parametrize('hardware_opt_in', [False, True])
def test_canonical_bringup_driver_start_does_not_imply_hardware_authority(
    inspect_launch, hardware_opt_in,
):
    overrides = {'start_arm_driver': 'true', 'start_gripper_driver': 'true'}
    if hardware_opt_in:
        overrides.update(
            arm_use_mock='false', arm_allow_motion_commands='true',
            gripper_allow_hardware_actuation='true',
        )
    module, _, context = inspect_launch(
        'dual_camera_foundation', 'system_bringup.launch.py', overrides,
    )
    drivers = _driver_parameters(module._launch_setup(context), context)
    assert drivers['arm_driver']['use_mock'] is not hardware_opt_in
    assert drivers['arm_driver']['allow_motion_commands'] is hardware_opt_in
    assert (
        drivers['gripper_driver']['allow_hardware_actuation']
        is hardware_opt_in
    )


@pytest.mark.parametrize('package, filename', [
    ('arm_console', 'arm_console_astra.launch.py'),
    ('wrist_camera_bringup', 'wrist_d405_console.launch.py'),
])
@pytest.mark.parametrize('hardware_opt_in', [False, True])
def test_camera_wrappers_forward_safety_values_to_final_drivers(
    inspect_launch, package, filename, hardware_opt_in,
):
    overrides = {'start_driver': 'true', 'start_gripper_driver': 'true'}
    if hardware_opt_in:
        overrides.update(use_mock='false', arm_allow_motion_commands='true',
                         gripper_allow_hardware_actuation='true')
    _, wrapper, wrapper_context = inspect_launch(package, filename, overrides)
    forwarded = [
        _include_arguments(action, wrapper_context)
        for action in wrapper.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    console_arguments = next(
        arguments for arguments in forwarded if 'use_mock' in arguments
    )
    _, console, context = inspect_launch(
        'arm_console', 'arm_console.launch.py', console_arguments,
    )
    drivers = _driver_parameters(console.entities, context)
    assert drivers['arm_driver']['use_mock'] is not hardware_opt_in
    assert drivers['arm_driver']['allow_motion_commands'] is hardware_opt_in
    assert (
        drivers['gripper_driver']['allow_hardware_actuation']
        is hardware_opt_in
    )


@pytest.mark.parametrize('real_feedback', [False, True])
def test_feedback_wrapper_remains_motion_disabled_in_both_modes(
    inspect_launch, real_feedback,
):
    overrides = {'arm_allow_motion_commands': 'true'}
    if real_feedback:
        overrides['arm_use_mock'] = 'false'
    _, wrapper, wrapper_context = inspect_launch(
        'dual_camera_foundation', 'arm_feedback_safe.launch.py', overrides,
    )
    include = next(action for action in wrapper.entities
                   if isinstance(action, IncludeLaunchDescription))
    arguments = _include_arguments(include, wrapper_context)
    module, _, context = inspect_launch(
        'dual_camera_foundation', 'system_bringup.launch.py', arguments,
    )
    drivers = _driver_parameters(module._launch_setup(context), context)
    assert drivers['arm_driver']['use_mock'] is not real_feedback
    assert drivers['arm_driver']['allow_motion_commands'] is False
    assert 'gripper_driver' not in drivers


@pytest.mark.parametrize('package, filename, arm_switch', [
    ('arm_console', 'arm_console.launch.py', 'start_driver'),
    ('dual_camera_foundation', 'system_bringup.launch.py', 'start_arm_driver'),
])
def test_gripper_opt_in_never_enables_arm_hardware_or_motion(
    inspect_launch, package, filename, arm_switch,
):
    module, description, context = inspect_launch(package, filename, {
        arm_switch: 'true',
        'start_gripper_driver': 'true',
        'gripper_allow_hardware_actuation': 'true',
    })
    actions = (
        module._launch_setup(context) if hasattr(module, '_launch_setup')
        else description.entities
    )
    drivers = _driver_parameters(actions, context)
    assert drivers['arm_driver']['use_mock'] is True
    assert drivers['arm_driver']['allow_motion_commands'] is False
    assert drivers['gripper_driver']['allow_hardware_actuation'] is True


@pytest.mark.parametrize('arm_mode', [None, 'real'])
def test_astra_preview_chain_never_grants_actuator_authority(
    inspect_launch, arm_mode,
):
    overrides = {} if arm_mode is None else {'arm_mode': arm_mode}
    module, _, context = inspect_launch(
        'dual_camera_foundation', 'astra_click_grasp.launch.py', overrides,
    )
    actions = module._launch_setup(context)
    arguments = next(
        arguments for arguments in (
            _include_arguments(action, context) for action in actions
            if isinstance(action, IncludeLaunchDescription)
        ) if 'arm_use_mock' in arguments
    )
    system, _, system_context = inspect_launch(
        'dual_camera_foundation', 'system_bringup.launch.py', arguments,
    )
    drivers = _driver_parameters(
        system._launch_setup(system_context), system_context,
    )
    assert drivers['arm_driver']['use_mock'] is (arm_mode is None)
    assert drivers['arm_driver']['allow_motion_commands'] is False
    assert 'gripper_driver' not in drivers
