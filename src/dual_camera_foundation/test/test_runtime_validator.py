import math

from dual_camera_foundation.runtime_validator import (
    FAIL,
    PASS,
    WARN,
    StreamMetadata,
    estimate_rate,
    evaluate_command_publishers,
    evaluate_rate,
    evaluate_registered_streams,
    evaluate_topic_publishers,
    point_distance,
    transform_xyz,
    validate_joint_solution,
)


def metadata(
    received_at=10.0,
    width=640,
    height=480,
    frame='camera_color_optical_frame',
    encoding='',
):
    return StreamMetadata(
        received_at=received_at,
        width=width,
        height=height,
        frame_id=frame,
        encoding=encoding,
    )


def test_rate_estimation_and_threshold_are_bounded():
    stamps = [10.0 + index * 0.05 for index in range(41)]
    assert math.isclose(estimate_rate(stamps), 20.0)
    result = evaluate_rate(
        'RGB',
        stamps,
        now=12.0,
        window_sec=3.0,
        expected_fps=30.0,
        minimum_fps_ratio=0.7,
        freshness_sec=1.0,
    )
    assert result.status == FAIL
    result = evaluate_rate(
        'RGB',
        stamps,
        now=12.0,
        window_sec=3.0,
        expected_fps=20.0,
        minimum_fps_ratio=0.7,
        freshness_sec=1.0,
    )
    assert result.status == PASS


def test_registered_stream_contract_requires_same_pixel_geometry():
    rgb = metadata(encoding='bgr8')
    depth = metadata(encoding='16UC1')
    info = metadata()
    result = evaluate_registered_streams(
        rgb,
        depth,
        info,
        640,
        480,
        'camera_color_optical_frame',
        now=10.1,
        freshness_sec=1.0,
    )
    assert result.status == PASS

    result = evaluate_registered_streams(
        rgb,
        metadata(width=320, encoding='16UC1'),
        info,
        640,
        480,
        'camera_color_optical_frame',
        now=10.1,
        freshness_sec=1.0,
    )
    assert result.status == FAIL
    assert 'size mismatch' in result.detail


def test_registered_stream_contract_rejects_unregistered_frame():
    result = evaluate_registered_streams(
        metadata(encoding='bgr8'),
        metadata(frame='camera_depth_optical_frame', encoding='16UC1'),
        metadata(),
        640,
        480,
        'camera_color_optical_frame',
        now=10.1,
        freshness_sec=1.0,
    )
    assert result.status == FAIL
    assert 'frame mismatch' in result.detail


def test_command_publishers_fail_closed_in_preview_mode():
    empty = evaluate_command_publishers({})
    assert empty.status == PASS

    console = evaluate_command_publishers({
        '/arm/joint_command': ('/arm_console',),
    })
    assert console.status == FAIL

    allowed_warning = evaluate_command_publishers(
        {'/arm/joint_command': ('/arm_console',)},
        require_zero=False,
    )
    assert allowed_warning.status == WARN

    visual_grasp = evaluate_command_publishers(
        {'/gripper/pulse_us': ('/visual_grasp_controller',)},
        require_zero=False,
    )
    assert visual_grasp.status == FAIL


def test_vendor_camera_may_have_two_endpoints_from_one_owner():
    endpoint = ('sensor_msgs/msg/Image', '/camera/camera')
    accepted = evaluate_topic_publishers(
        '/camera/color/image_raw',
        (endpoint, endpoint),
        'sensor_msgs/msg/Image',
        '/camera/camera',
        require_exactly_one=False,
    )
    assert accepted.status == PASS

    duplicate_owner = evaluate_topic_publishers(
        '/camera/color/image_raw',
        (endpoint, ('sensor_msgs/msg/Image', '/second_camera')),
        'sensor_msgs/msg/Image',
        '/camera/camera',
        require_exactly_one=False,
    )
    assert duplicate_owner.status == FAIL


def test_joint_solution_must_be_complete_ordered_and_finite():
    names = [f'joint{index}' for index in range(1, 7)]
    assert validate_joint_solution(names, [0.0] * 6).status == PASS
    assert validate_joint_solution(names[::-1], [0.0] * 6).status == FAIL
    values = [0.0] * 5 + [float('nan')]
    assert validate_joint_solution(names, values).status == FAIL
    assert validate_joint_solution(names, [0.0] * 5 + [3.2]).status == FAIL
    unlocked_j6 = validate_joint_solution(
        names, [0.0] * 5 + [math.radians(1.0)]
    )
    assert unlocked_j6.status == PASS


def test_rigid_transform_replay_and_distance():
    half = math.sqrt(0.5)
    transformed = transform_xyz(
        (1.0, 0.0, 0.0),
        (1.0, 2.0, 3.0),
        (0.0, 0.0, half, half),
    )
    assert point_distance(transformed, (1.0, 3.0, 3.0)) < 1.0e-12
    assert math.isinf(point_distance((0.0, 0.0, 0.0), (0.0, 0.0, math.nan)))
