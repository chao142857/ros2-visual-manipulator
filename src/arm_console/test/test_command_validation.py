import math
from queue import SimpleQueue
import threading
from types import SimpleNamespace

import pytest

from arm_console.ros_bridge import (
    ArmConsoleNode,
    BackendStatus,
    backend_transition_events,
    classify_backend,
    live_follow_block_reason,
    live_runtime_block_reason,
    move_block_reason,
    validate_gripper_pulse,
    validate_live_timing,
    validate_target_degrees,
    validate_timeout_sec,
)


LIMIT_MIN = [-3.14] * 6
LIMIT_MAX = [3.14] * 6


def test_valid_degree_targets_are_converted_to_radians():
    values = [-179.9, -90.0, 0.0, 45.0, 90.0, 0.0]

    result = validate_target_degrees(values, LIMIT_MIN, LIMIT_MAX)

    assert result == pytest.approx(
        tuple(math.radians(value) for value in values)
    )


def test_preview_target_accepts_nonzero_j6():
    result = validate_target_degrees(
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        LIMIT_MIN,
        LIMIT_MAX,
    )
    assert result[5] == pytest.approx(math.radians(1.0))


def test_target_above_measured_limit_is_rejected():
    with pytest.raises(ValueError, match='J5 must be between'):
        validate_target_degrees(
            [0.0, 0.0, 0.0, 0.0, 180.0, 0.0],
            LIMIT_MIN,
            LIMIT_MAX,
        )


def test_continuous_j6_is_not_position_limited():
    result = validate_target_degrees(
        [0.0, 0.0, 0.0, 0.0, 0.0, 720.0],
        LIMIT_MIN,
        LIMIT_MAX,
    )
    assert result[5] == pytest.approx(4.0 * math.pi)


def test_backend_requires_fresh_state_and_all_control_endpoints():
    assert classify_backend(None, 1.0, True, True, True) == (
        'waiting',
        'never',
    )
    assert classify_backend(1.1, 1.0, True, True, True) == (
        'feedback_lost',
        'stale',
    )
    assert classify_backend(0.1, 1.0, True, False, True) == (
        'degraded',
        'fresh',
    )
    assert classify_backend(0.1, 1.0, True, True, True) == (
        'ready',
        'fresh',
    )
    assert classify_backend(0.1, 1.0, True, True, True, True) == (
        'degraded',
        'fresh',
    )


def test_gripper_pulse_validation():
    assert validate_gripper_pulse(600, 600, 1700) == 600
    assert validate_gripper_pulse(1200, 600, 1700) == 1200
    assert validate_gripper_pulse(1700, 600, 1700) == 1700
    with pytest.raises(ValueError, match='between 600 and 1700'):
        validate_gripper_pulse(599, 600, 1700)


@pytest.mark.parametrize(
    ('publish_period', 'heartbeat_timeout', 'expected_message'),
    [
        (math.inf, 0.25, 'live_publish_period_sec must be finite'),
        (0.1, math.inf, 'live_gui_heartbeat_timeout_sec must be finite'),
        (0.01, 0.25, 'live_publish_period_sec must be between'),
        (0.1, 1.1, 'must be no greater than 1.0 second'),
    ],
)
def test_live_timing_rejects_non_finite_or_unreasonable_values(
    publish_period,
    heartbeat_timeout,
    expected_message,
):
    with pytest.raises(ValueError, match=expected_message):
        validate_live_timing(publish_period, heartbeat_timeout)


def test_live_timing_requires_two_publish_periods_for_gui_deadman():
    with pytest.raises(ValueError, match='allow two publish periods'):
        validate_live_timing(0.1, 0.19)

    assert validate_live_timing(0.1, 0.25) == (0.1, 0.25)


@pytest.mark.parametrize(
    ('parameter_name', 'value'),
    [
        ('state_timeout_sec', math.nan),
        ('state_timeout_sec', math.inf),
        ('service_timeout_sec', math.nan),
        ('service_timeout_sec', math.inf),
    ],
)
def test_console_timeouts_reject_nan_and_infinity(parameter_name, value):
    with pytest.raises(
        ValueError,
        match=f'{parameter_name} must be finite and positive',
    ):
        validate_timeout_sec(value, parameter_name)


def test_gripper_or_rviz_change_does_not_repeat_arm_feedback_lost_event():
    lost = BackendStatus(
        aggregate='feedback_lost',
        state_stream='stale',
        state_age_sec=1.1,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        gripper_ready=False,
        preview_ready=False,
        state_publishers=1,
    )
    route_change = BackendStatus(
        **{
            **lost.__dict__,
            'gripper_ready': True,
            'preview_ready': True,
            'rviz_detected': True,
        }
    )

    events = backend_transition_events(lost, route_change)

    assert events == ((
        'info',
        'GRIPPER ROUTE: command subscriber detected',
    ),)
    assert all('FEEDBACK LOST' not in message for _level, message in events)


def test_move_sender_blocks_feedback_recovery():
    recovering = BackendStatus(
        aggregate='recovering',
        state_stream='recovering',
        state_age_sec=0.0,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        recovery_frames=2,
        recovery_frames_required=3,
    )

    reason = move_block_reason(recovering)

    assert '3 consecutive valid frames' in reason
    assert '(2/3)' in reason


def test_explicit_move_is_blocked_while_live_follow_is_armed():
    status = BackendStatus(
        aggregate='ready',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=True,
        live_heartbeat_ready=True,
        live_armed=True,
    )

    assert 'LIVE FOLLOW is armed' in move_block_reason(status)


def test_live_follow_requires_all_dedicated_routes():
    status = BackendStatus(
        aggregate='ready',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=False,
    )

    assert 'state publisher' in live_follow_block_reason(status)


def test_live_target_slot_keeps_only_the_latest_six_axis_target():
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._live_target_lock = threading.Lock()
    node._live_target = None
    node._live_gui_heartbeat_at = None
    node._live_armed = True
    node._live_safety_trip_latched = False

    first = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    latest = (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0)
    node.request_live_target(first)
    first_heartbeat = node._live_gui_heartbeat_at
    node.request_live_target(latest)

    assert node._live_target == latest
    assert node._live_gui_heartbeat_at >= first_heartbeat


def test_armed_live_feedback_loss_stops_heartbeat_and_requests_disarm():
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._live_armed = True
    node._live_target_lock = threading.Lock()
    node._live_target = (0.0,) * 6
    node._live_gui_heartbeat_at = 123.0
    node._live_safety_trip_latched = False
    node._live_safety_disarm_requested = False
    node._pending_services = {}

    stale = BackendStatus(
        aggregate='feedback_lost',
        state_stream='stale',
        state_age_sec=1.1,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        state_publishers=1,
        state_source_ready=True,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=True,
        live_heartbeat_ready=True,
        live_armed=True,
    )
    node._build_status = lambda: stale

    heartbeat_messages = []
    target_messages = []
    log_messages = []
    disarm_requests = []
    node.live_heartbeat_publisher = SimpleNamespace(
        publish=heartbeat_messages.append,
    )
    node.live_command_publisher = SimpleNamespace(
        publish=target_messages.append,
    )
    node.signals = SimpleNamespace(
        log_message=SimpleNamespace(
            emit=lambda level, message: log_messages.append(
                (level, message)
            ),
        ),
    )
    node._call_live_follow = (
        lambda enabled, baseline: disarm_requests.append(
            (enabled, baseline)
        )
    )

    node._publish_live_target()
    node._publish_live_target()

    assert heartbeat_messages == []
    assert target_messages == []
    assert node._live_target is None
    assert node._live_gui_heartbeat_at is None
    assert disarm_requests == [(False, None)]
    assert len(log_messages) == 1
    assert 'ARM feedback is stale' in log_messages[0][1]


def test_successful_disarm_service_clears_trip_without_state_callback():
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._live_armed = True
    node._live_state_seen = True
    node._live_target_lock = threading.Lock()
    node._live_target = (0.0,) * 6
    node._live_gui_heartbeat_at = 123.0
    node._live_safety_trip_latched = True
    node._live_safety_disarm_requested = True
    node._live_disarmed_event = threading.Event()

    response = SimpleNamespace(success=True, message='disarmed')
    future = SimpleNamespace(result=lambda: response)
    node._pending_services = {'live_disable': (future, 456.0)}
    motion_events = []
    operation_events = []
    node.signals = SimpleNamespace(
        motion_status_changed=SimpleNamespace(emit=motion_events.append),
        operation_finished=SimpleNamespace(
            emit=lambda *event: operation_events.append(event),
        ),
    )

    node._live_service_completed('live_disable', False, future)

    assert not node._live_armed
    assert node._live_disarmed_event.is_set()
    assert not node._live_safety_trip_latched
    assert not node._live_safety_disarm_requested
    assert node._live_target is None
    assert node._live_gui_heartbeat_at is None
    assert operation_events[-1][0:2] == ('live_disable', True)

    node._live_armed = True
    node.request_live_heartbeat()
    assert node._live_gui_heartbeat_at is not None


def test_unavailable_disarm_service_keeps_trip_latched():
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._live_armed = True
    node._live_state_seen = True
    node._live_target_lock = threading.Lock()
    node._live_target = (0.0,) * 6
    node._live_gui_heartbeat_at = 123.0
    node._live_safety_trip_latched = True
    node._live_safety_disarm_requested = True
    node._live_disarmed_event = threading.Event()
    node._pending_services = {}
    node._request_queue = SimpleQueue()
    node._safety_action_lock = threading.Lock()
    node._safety_actions_pending = set()
    node.live_client = SimpleNamespace(service_is_ready=lambda: False)
    operation_events = []
    log_events = []
    node.signals = SimpleNamespace(
        operation_finished=SimpleNamespace(
            emit=lambda *event: operation_events.append(event),
        ),
        log_message=SimpleNamespace(
            emit=lambda *event: log_events.append(event),
        ),
    )

    node._call_live_follow(False, None)

    assert node._live_armed
    assert not node._live_disarmed_event.is_set()
    assert node._live_safety_trip_latched
    assert node._live_safety_disarm_requested
    assert node._live_target is None
    assert node._live_gui_heartbeat_at is None

    node.request_live_heartbeat()
    node.request_live_mode(True, (0.0,) * 6)
    node._drain_requests()

    assert node._live_armed
    assert node._live_safety_trip_latched
    assert node._live_gui_heartbeat_at is None
    assert len(operation_events) == 2
    assert operation_events[-1] == (
        'live_enable',
        False,
        'LIVE FOLLOW blocked: driver still reports armed; disarm first',
    )


def test_operator_safety_action_cancels_queued_live_arm():
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._request_queue = SimpleQueue()
    node._pending_services = {}
    node._safety_action_lock = threading.Lock()
    node._safety_actions_pending = set()
    node._live_target_lock = threading.Lock()
    node._live_target = None
    node._live_gui_heartbeat_at = None
    node._live_armed = False
    node._live_safety_trip_latched = True
    node._live_safety_disarm_requested = True
    operation_events = []
    node.signals = SimpleNamespace(
        operation_finished=SimpleNamespace(
            emit=lambda *event: operation_events.append(event),
        ),
    )

    baseline = (0.0,) * 6
    node.request_live_mode(True, baseline)
    node.request_stop()
    node._call_live_follow(True, baseline)

    assert node._live_safety_trip_latched
    assert node._live_target is None
    assert node._live_gui_heartbeat_at is None
    assert operation_events == [(
        'live_enable',
        False,
        'LIVE FOLLOW blocked: STOP or DISABLE is pending',
    )]


@pytest.mark.parametrize(
    ('request_name', 'operation'),
    [
        ('request_stop', 'stop'),
        ('request_disable', 'disable'),
    ],
)
def test_queued_safety_action_blocks_later_move_and_live_arm(
    request_name,
    operation,
):
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._request_queue = SimpleQueue()
    node._pending_services = {}
    node._safety_action_lock = threading.Lock()
    node._safety_actions_pending = set()
    node._live_target_lock = threading.Lock()
    node._live_target = None
    node._live_gui_heartbeat_at = None
    node._live_safety_trip_latched = False
    node._live_safety_disarm_requested = False
    operation_events = []
    trigger_calls = []
    command_messages = []
    node.signals = SimpleNamespace(
        operation_finished=SimpleNamespace(
            emit=lambda *event: operation_events.append(event),
        ),
    )
    node.command_publisher = SimpleNamespace(
        publish=command_messages.append,
    )
    node._call_trigger = (
        lambda name, client: trigger_calls.append((name, client))
    )
    node.stop_client = object()
    node.disable_client = object()

    getattr(node, request_name)()
    node.request_move((0.0,) * 6)
    node.request_live_mode(True, (0.0,) * 6)
    node._drain_requests()

    assert trigger_calls == [(
        operation,
        node.stop_client if operation == 'stop' else node.disable_client,
    )]
    assert command_messages == []
    assert (
        'move',
        False,
        'MOVE blocked: STOP or DISABLE is pending',
    ) in operation_events
    assert (
        'live_enable',
        False,
        'LIVE FOLLOW blocked: STOP or DISABLE is pending',
    ) in operation_events


def test_pending_live_disarm_blocks_move_and_rearm():
    node = ArmConsoleNode.__new__(ArmConsoleNode)
    node._pending_services = {'live_disable': (object(), 123.0)}
    node._safety_action_lock = threading.Lock()
    node._safety_actions_pending = set()
    operation_events = []
    command_messages = []
    node.signals = SimpleNamespace(
        operation_finished=SimpleNamespace(
            emit=lambda *event: operation_events.append(event),
        ),
    )
    node.command_publisher = SimpleNamespace(
        publish=command_messages.append,
    )

    node._publish_move((0.0,) * 6)
    node._call_live_follow(True, (0.0,) * 6)

    assert command_messages == []
    assert operation_events == [
        (
            'move',
            False,
            'MOVE blocked: LIVE FOLLOW disarm is pending',
        ),
        (
            'live_enable',
            False,
            'LIVE FOLLOW blocked: disarm is pending',
        ),
    ]


def test_live_runtime_route_conflict_is_fail_closed():
    status = BackendStatus(
        aggregate='degraded',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=False,
        command_subscribers=2,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        state_publishers=1,
        state_source_ready=True,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=True,
        live_heartbeat_ready=True,
        live_armed=True,
    )

    assert 'control route conflict' in live_runtime_block_reason(status)


def test_external_live_heartbeat_publisher_blocks_arm_and_runtime():
    status = BackendStatus(
        aggregate='ready',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        state_publishers=1,
        state_source_ready=True,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=True,
        live_heartbeat_ready=True,
        external_live_heartbeat_publishers=1,
    )

    assert 'heartbeat publisher' in live_follow_block_reason(status)
    assert 'heartbeat route conflict' in live_runtime_block_reason(status)


def test_multiple_driver_routes_block_move_and_live_follow():
    status = BackendStatus(
        aggregate='degraded',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=False,
        command_subscribers=2,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        state_publishers=2,
        state_source_ready=False,
        live_command_ready=False,
        live_command_subscribers=2,
        live_service_ready=True,
        live_state_ready=False,
        live_state_publishers=2,
        live_heartbeat_ready=False,
        live_heartbeat_subscribers=2,
    )

    assert 'multiple /joint_states' in move_block_reason(status)
    assert 'multiple /joint_states' in live_follow_block_reason(status)
