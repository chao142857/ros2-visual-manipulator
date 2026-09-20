import importlib.util
from pathlib import Path
import re
from types import SimpleNamespace

from launch import LaunchContext
import pytest


PACKAGE = Path(__file__).parents[1]


def load_astra_click_grasp_launch():
    path = PACKAGE / 'launch' / 'astra_click_grasp.launch.py'
    spec = importlib.util.spec_from_file_location(
        'astra_click_grasp_launch_under_test',
        path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_canonical_system_hardware_drivers_are_opt_in():
    text = (PACKAGE / 'launch' / 'system_bringup.launch.py').read_text()
    for argument in (
        'start_arm_driver',
        'start_gripper_driver',
        'start_astra',
        'start_d405',
    ):
        pattern = (
            r"DeclareLaunchArgument\('" + re.escape(argument)
            + r"', default_value='false'\)"
        )
        assert re.search(pattern, text)


def test_astra_transform_switch_reaches_runtime_and_keeps_tags_independent():
    system = (PACKAGE / 'launch' / 'system_bringup.launch.py').read_text()
    assert re.search(
        r"start_astra_transform\s*=\s*_as_bool\(\s*"
        r"value\('start_astra_transform'\)",
        system,
    )
    assert (
        "'start_transform': str(start_astra_transform).lower()"
        in system
    )

    runtime = (
        PACKAGE.parents[0]
        / 'camera_perception'
        / 'launch'
        / 'camera_runtime.launch.py'
    ).read_text()
    assert re.search(
        r"DeclareLaunchArgument\(\s*'start_transform',"
        r"\s*default_value='true'",
        runtime,
    )
    assert runtime.count('condition=IfCondition(start_transform)') == 2
    assert re.search(
        r"tag_localizer\s*=\s*Node\(.*?"
        r"condition=IfCondition\(start_tag_localizer\)",
        runtime,
        re.DOTALL,
    )


def test_d405_chain_uses_tight_nearest_aligned_depth_without_driver():
    text = (
        PACKAGE / 'launch' / 'd405_rgbd_perception.launch.py'
    ).read_text()
    assert 'aligned_depth_to_color/image_raw' in text
    assert 'aligned_depth_to_color/camera_info' in text
    assert "'max_depth_age_sec'" in text
    assert "default_value='0.05'" in text
    assert "'depth_queue_size'" in text
    assert "package='realsense2_camera'" not in text
    assert "package='arm_driver'" not in text


def test_base_frame_d405_transform_requires_validated_mount_contract():
    text = (PACKAGE / 'launch' / 'system_bringup.launch.py').read_text()
    assert 'validate_mount_contract(' in text
    assert 'start_d405_transform' in text
    assert 'use_mount_calibration' in text
    assert re.search(
        r"DeclareLaunchArgument\(\s*'start_d405_transform',"
        r"\s*default_value='false'",
        text,
    )
    assert "'start_transform': str(start_d405_transform).lower()" in text

    direct_text = (
        PACKAGE / 'launch' / 'dual_perception.launch.py'
    ).read_text()
    assert 'validate_mount_contract(' in direct_text
    assert re.search(
        r"DeclareLaunchArgument\(\s*'start_d405_transform',"
        r"\s*default_value='false'",
        direct_text,
    )


def test_refiner_launch_exposes_cross_camera_and_jump_gates():
    text = (PACKAGE / 'launch' / 'system_bringup.launch.py').read_text()
    for parameter in (
        'refiner_max_pair_time_delta_sec',
        'refiner_max_pair_distance_m',
        'refiner_max_output_jump_m',
        'refiner_require_astra_reference',
        'refiner_require_same_frame',
    ):
        assert parameter in text


def test_all_legacy_system_entry_points_use_known_package_names():
    launch_dir = PACKAGE / 'launch'
    combined = '\n'.join(
        path.read_text() for path in launch_dir.glob('*.launch.py')
    )
    assert 'dual_camera_tools' not in combined


def test_astra_click_grasp_entry_is_preview_only():
    text = (
        PACKAGE / 'launch' / 'astra_click_grasp.launch.py'
    ).read_text()
    assert re.search(
        r'arm_allow_motion_commands.{0,8}false',
        text,
    )
    bringup_text = (
        PACKAGE / 'launch' / 'system_bringup.launch.py'
    ).read_text()
    assert re.search(
        r'allow_motion_commands.{0,30}_as_bool',
        bringup_text,
    )
    assert re.search(
        r'value\(.arm_allow_motion_commands.\)',
        bringup_text,
    )
    assert text.count('def generate_launch_description') == 1
    assert "'arm_mode'," in text
    assert "default_value='mock'" in text
    assert "'start_astra': 'true'" in text
    assert "'start_gripper_driver': 'false'" in text
    assert "'start_console': 'false'" in text
    assert "'enable_optical_flow_tracking': 'false'" in text
    assert "'execute_hardware': False" in text
    assert "'require_zero_command_publishers': True" in text
    assert "DeclareLaunchArgument('execute_hardware'" not in text
    assert 'event.returncode == 0' in text
    assert 'RaiseError' not in text
    assert 'function=_raise_validator_failure' in text
    setup_text = (PACKAGE / 'setup.py').read_text()
    assert "'astra_click_grasp_validator = '" in setup_text
    assert "'dual_camera_foundation.runtime_validator:main'" in setup_text


def test_astra_click_grasp_imports_on_humble_and_propagates_failure():
    module = load_astra_click_grasp_launch()
    description = module.generate_launch_description()
    assert description is not None

    actions = module._validator_exit_actions(
        SimpleNamespace(returncode=7),
        None,
    )
    assert len(actions) == 1
    with pytest.raises(RuntimeError, match='return code 7'):
        actions[0].execute(LaunchContext())


def test_canonical_astra_geometry_is_explicit():
    text = (PACKAGE / 'launch' / 'system_bringup.launch.py').read_text()
    expected_pairs = (
        "'depth_registration': 'true'",
        "'color_depth_synchronization': 'true'",
        "'color_width': '640'",
        "'color_height': '480'",
        "'color_fps': '30'",
        "'depth_width': '640'",
        "'depth_height': '480'",
        "'depth_fps': '30'",
    )
    for pair in expected_pairs:
        assert pair in text
