import math
from types import SimpleNamespace

import arm_driver.arm_driver_node as driver_module
import pytest
from arm_driver.joint_mapping import raw_deg_to_ideal_rad

from test_start_handshake import FakeSerial
from test_start_handshake import make_node


JOINT_NAMES = [f'joint{index}' for index in range(1, 7)]
BASELINE_IDEAL = raw_deg_to_ideal_rad(
    [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
)


class FakeClock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


class FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(bool(message.data))


def set_clock(node, nanoseconds=10_000_000_000):
    clock = FakeClock(nanoseconds)
    node.get_clock = lambda: clock
    return clock


def response():
    return SimpleNamespace(success=False, message='')


def live_request(enabled):
    return SimpleNamespace(data=enabled)


def stamped_header(stamp_ns):
    return SimpleNamespace(
        stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )
    )


def heartbeat(stamp_ns):
    return stamped_header(stamp_ns)


def live_message(stamp_ns, joint='joint1', position_rad=0.0):
    positions = list(BASELINE_IDEAL)
    positions[JOINT_NAMES.index(joint)] = position_rad
    return SimpleNamespace(
        header=stamped_header(stamp_ns),
        name=list(JOINT_NAMES),
        position=positions,
    )


def full_live_message(stamp_ns, positions, names=None):
    if names is None:
        names = JOINT_NAMES
    return SimpleNamespace(
        header=stamped_header(stamp_ns),
        name=list(names),
        position=list(positions),
    )


def arm_node(responses=()):
    node = make_node(responses)
    node.current_ideal_radians = raw_deg_to_ideal_rad(
        [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
    )
    node.live_follow_state_publisher = FakePublisher()
    node._last_live_state_published = False
    clock = set_clock(node)
    return node, clock


def set_active(node, clock, heartbeat_ready=True):
    node.controller_started = True
    node._activate_live_follow()
    assert node.live_follow_active
    clock.nanoseconds += 1
    if heartbeat_ready:
        node.live_follow_heartbeat_callback(heartbeat(clock.nanoseconds))
        clock.nanoseconds += 1


def test_live_arm_is_stop_start_mode_probe_and_sends_no_target():
    node, _clock = arm_node([
        b'Stopped ok\r\n',
        b'Started ok\r\n',
        b'ok Set command mode to [2]\r\n',
        b'ok 0 -75 180 0 0 0\r\n',
    ])

    result = node.live_follow_callback(live_request(True), response())

    assert result.success
    assert node.live_follow_active
    assert node.controller_started
    assert node.serial_port.writes == [
        b'!STOP\n',
        b'!START\n',
        b'#CMDMODE 2\n',
        b'#GETJPOS\n',
    ]
    assert not any(write.startswith(b'&') for write in node.serial_port.writes)
    assert node.live_follow_state_publisher.values == [True]
    assert math.degrees(node.current_ideal_radians[1]) == pytest.approx(75.0)


def test_motion_interlock_rejects_live_arm_without_serial_commands():
    node, _clock = arm_node()
    node.allow_motion_commands = False

    result = node.live_follow_callback(live_request(True), response())

    assert not result.success
    assert 'disabled' in result.message.lower()
    assert not node.live_follow_active
    assert node.serial_port.writes == []


def test_armed_heartbeat_refreshes_deadman_without_sending_move():
    node, clock = arm_node([
        b'Stopped ok\r\n',
        b'Started ok\r\n',
        b'ok Set command mode to [2]\r\n',
        b'ok 0 -75 180 0 0 0\r\n',
    ])
    assert node.live_follow_callback(live_request(True), response()).success
    clock.nanoseconds += 1

    node.live_follow_heartbeat_callback(heartbeat(clock.nanoseconds))

    assert node.live_last_heartbeat_monotonic is not None
    assert node.pending_live_target is None
    assert not any(
        write.startswith(b'&') for write in node.serial_port.writes
    )


def test_old_epoch_heartbeat_is_rejected_without_refreshing_deadman():
    node, clock = arm_node()
    set_active(node, clock, heartbeat_ready=False)
    armed_at = node.live_armed_monotonic

    node.live_follow_heartbeat_callback(
        heartbeat(node.live_epoch_stamp_ns - 1)
    )

    assert node.live_last_heartbeat_stamp_ns is None
    assert node.live_last_heartbeat_monotonic is None
    assert node.live_armed_monotonic == armed_at


def test_live_target_is_clamped_to_one_degree_and_has_explicit_speed():
    node, clock = arm_node([b'16\r\n'])
    set_active(node, clock)
    desired = math.radians(30.0)

    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=desired)
    )
    node.process_live_follow()

    assert node.serial_port.writes == [
        b'&-1.00,-75.00,180.00,0.00,0.00,0.00,10.00\n'
    ]
    assert node.live_follow_active


def test_live_target_waits_for_first_valid_epoch_heartbeat():
    node, clock = arm_node([b'16\r\n'])
    set_active(node, clock, heartbeat_ready=False)
    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(20.0))
    )
    pending = node.pending_live_target

    with node.serial_lock:
        assert not node._send_live_target_locked(
            pending,
            driver_module.time.monotonic(),
        )
    node.process_live_follow()

    assert node.serial_port.writes == []
    assert node.pending_live_target == pending

    clock.nanoseconds += 1
    node.live_follow_heartbeat_callback(heartbeat(clock.nanoseconds))
    node.process_live_follow()

    assert node.serial_port.writes == [
        b'&-1.00,-75.00,180.00,0.00,0.00,0.00,10.00\n'
    ]


def test_live_target_reorders_all_six_named_joints():
    node, clock = arm_node()
    set_active(node, clock)
    desired = list(BASELINE_IDEAL)
    desired[0] = math.radians(10.0)
    reversed_names = list(reversed(JOINT_NAMES))
    reversed_positions = list(reversed(desired))

    node.live_joint_command_callback(
        full_live_message(
            clock.nanoseconds,
            reversed_positions,
            reversed_names,
        )
    )

    assert node.pending_live_target == (
        clock.nanoseconds,
        tuple(desired),
    )


def test_live_target_rejects_missing_or_duplicate_joint_names():
    node, clock = arm_node()
    set_active(node, clock)
    duplicate_names = list(JOINT_NAMES)
    duplicate_names[-1] = 'joint5'

    node.live_joint_command_callback(
        full_live_message(
            clock.nanoseconds,
            BASELINE_IDEAL,
            duplicate_names,
        )
    )

    assert node.pending_live_target is None


def test_multi_axis_target_keeps_all_axes_converging_from_fresh_feedback():
    node, clock = arm_node([b'16\r\n', b'16\r\n'])
    set_active(node, clock)
    desired = list(BASELINE_IDEAL)
    desired[0] = math.radians(20.0)
    desired[1] = math.radians(60.0)

    node.live_joint_command_callback(
        full_live_message(clock.nanoseconds, desired)
    )
    node.process_live_follow()

    assert node.serial_port.writes == [
        b'&-1.00,-74.00,180.00,0.00,0.00,0.00,10.00\n'
    ]

    node.current_ideal_radians = raw_deg_to_ideal_rad(
        [-1.0, -74.0, 180.0, 0.0, 0.0, 0.0]
    )
    node.last_feedback_monotonic += 0.01
    node.live_last_send_monotonic = None
    node.process_live_follow()

    assert node.serial_port.writes[-1] == (
        b'&-2.00,-73.00,180.00,0.00,0.00,0.00,10.00\n'
    )


def test_many_targets_are_coalesced_to_only_the_latest():
    node, clock = arm_node([b'15\r\n'])
    set_active(node, clock)

    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(5.0))
    )
    clock.nanoseconds += 1
    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(20.0))
    )
    node.process_live_follow()

    assert len(node.serial_port.writes) == 1
    assert node.serial_port.writes[0].startswith(b'&-1.00,')


def test_next_bounded_step_waits_for_newer_feedback():
    node, clock = arm_node([b'16\r\n', b'16\r\n'])
    set_active(node, clock)

    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(20.0))
    )
    node.process_live_follow()
    assert len(node.serial_port.writes) == 1

    node.live_last_send_monotonic = None
    node.process_live_follow()

    assert len(node.serial_port.writes) == 1
    assert node.pending_live_target is not None

    node.last_feedback_monotonic += 0.01
    node.current_ideal_radians[0] = math.radians(0.5)
    node.process_live_follow()

    assert len(node.serial_port.writes) == 2
    assert node.serial_port.writes[-1].startswith(b'&-1.50,')


def test_same_formatted_motion_target_is_never_written_twice():
    node, clock = arm_node([b'16\r\n'])
    set_active(node, clock)

    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(20.0))
    )
    node.process_live_follow()
    assert len(node.serial_port.writes) == 1

    node.last_feedback_monotonic += 0.01
    node.live_last_send_monotonic = None
    node.process_live_follow()

    assert len(node.serial_port.writes) == 1


def test_old_epoch_and_stale_targets_do_not_refresh_heartbeat():
    node, clock = arm_node()
    set_active(node, clock, heartbeat_ready=False)
    epoch = node.live_epoch_stamp_ns

    node.live_joint_command_callback(
        live_message(epoch - 1, position_rad=math.radians(5.0))
    )
    assert node.pending_live_target is None
    assert node.live_last_heartbeat_monotonic is None

    clock.nanoseconds = epoch + 1_000_000_000
    node.live_joint_command_callback(
        live_message(epoch + 1, position_rad=math.radians(5.0))
    )
    assert node.pending_live_target is None
    assert node.live_last_heartbeat_monotonic is None


def test_out_of_order_target_is_not_allowed_to_replace_latest():
    node, clock = arm_node()
    set_active(node, clock)
    newest_stamp = clock.nanoseconds + 10
    clock.nanoseconds = newest_stamp
    node.live_joint_command_callback(
        live_message(newest_stamp, position_rad=math.radians(5.0))
    )
    expected = node.pending_live_target

    node.live_joint_command_callback(
        live_message(newest_stamp - 1, position_rad=math.radians(20.0))
    )

    assert node.pending_live_target == expected


def test_zero_delta_never_writes_move_or_refreshes_deadman():
    node, clock = arm_node()
    set_active(node, clock)
    heartbeat_at = node.live_last_heartbeat_monotonic

    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=0.0)
    )
    node.process_live_follow()

    assert node.serial_port.writes == []
    assert node.live_last_heartbeat_monotonic == heartbeat_at
    assert node.live_follow_active


def test_watchdog_invalidates_epoch_before_stop(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(
        driver_module.time,
        'monotonic',
        lambda: now[0],
    )
    node, clock = arm_node([b'Stopped ok\r\n'])
    set_active(node, clock)
    node.live_follow_tracking = True
    node.live_last_heartbeat_monotonic = now[0]
    node.pending_live_target = (
        clock.nanoseconds,
        tuple(BASELINE_IDEAL),
    )
    public_state_during_stop = []
    original_read = node._read_matching_response_locked

    def observe_public_state(pattern, timeout):
        public_state_during_stop.append(
            node.live_follow_state_publisher.values[-1]
        )
        return original_read(pattern, timeout)

    node._read_matching_response_locked = observe_public_state
    now[0] += node.live_watchdog_timeout_sec + 0.01

    node.process_live_follow()

    assert not node.live_follow_active
    assert node.live_epoch_stamp_ns is None
    assert node.pending_live_target is None
    assert node.serial_port.writes == [b'!STOP\n']
    assert not node.controller_started
    assert public_state_during_stop == [True]
    assert node.live_follow_state_publisher.values == [True, False]
    barrier_ns = node.minimum_command_stamp_ns
    assert barrier_ns == clock.nanoseconds
    stale_stamp = SimpleNamespace(
        sec=(barrier_ns - 1) // 1_000_000_000,
        nanosec=(barrier_ns - 1) % 1_000_000_000,
    )
    assert not node.send_joint_command(
        '&1,2,3,4,5,0,30\n',
        command_stamp=stale_stamp,
    )
    assert node.serial_port.writes == [b'!STOP\n']


def test_watchdog_stops_if_no_first_heartbeat_arrives(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(
        driver_module.time,
        'monotonic',
        lambda: now[0],
    )
    node, clock = arm_node([b'Stopped ok\r\n'])
    set_active(node, clock, heartbeat_ready=False)
    now[0] += node.live_watchdog_timeout_sec + 0.01

    node.process_live_follow()

    assert not node.live_follow_active
    assert node.serial_port.writes == [b'!STOP\n']


def test_live_start_ack_failure_attempts_stop_before_disconnect():
    node, _clock = arm_node([
        b'Stopped ok\r\n',
        b'not-started\r\n',
        b'Stopped ok\r\n',
    ])
    serial_port = node.serial_port

    result = node.live_follow_callback(live_request(True), response())

    assert not result.success
    assert serial_port.writes == [
        b'!STOP\n',
        b'!START\n',
        b'!STOP\n',
    ]
    assert serial_port.closed
    assert not node.live_follow_active


def test_live_queue_backlog_stops_and_requires_explicit_rearm():
    node, clock = arm_node([b'14\r\n', b'Stopped ok\r\n'])
    set_active(node, clock)
    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(10.0))
    )

    node.process_live_follow()

    assert node.serial_port.writes == [
        b'&-1.00,-75.00,180.00,0.00,0.00,0.00,10.00\n',
        b'!STOP\n',
    ]
    assert not node.live_follow_active
    assert node.serial_port.is_open

    node.process_live_follow()
    assert len(node.serial_port.writes) == 2


def test_first_poll_failure_during_live_sends_stop_immediately():
    node, clock = arm_node([b'Stopped ok\r\n'])
    set_active(node, clock)

    with node.serial_lock:
        node._record_poll_failure_locked('missing GETJPOS')

    assert not node.live_follow_active
    assert node.serial_port.writes == [b'!STOP\n']
    assert node.consecutive_poll_failures == 1
    assert node.serial_port.is_open


def test_operator_exit_clears_pending_before_stop():
    node, clock = arm_node([b'Stopped ok\r\n'])
    set_active(node, clock)
    node.pending_live_target = (
        clock.nanoseconds,
        tuple(BASELINE_IDEAL),
    )
    public_state_during_stop = []
    original_read = node._read_matching_response_locked

    def observe_public_state(pattern, timeout):
        public_state_during_stop.append(
            node.live_follow_state_publisher.values[-1]
        )
        return original_read(pattern, timeout)

    node._read_matching_response_locked = observe_public_state

    result = node.live_follow_callback(live_request(False), response())

    assert result.success
    assert not node.live_follow_active
    assert node.pending_live_target is None
    assert node.serial_port.writes == [b'!STOP\n']
    assert public_state_during_stop == [True]
    assert node.live_follow_state_publisher.values == [True, False]


def test_stop_keeps_public_live_true_until_ack_then_publishes_false():
    node, clock = arm_node([b'Stopped ok\r\n'])
    set_active(node, clock)
    state_during_ack_wait = []
    original_read = node._read_matching_response_locked

    def observe_state_before_ack(pattern, timeout):
        state_during_ack_wait.append(
            (
                node.live_follow_active,
                node.pending_live_target,
                node.live_follow_state_publisher.values[-1],
            )
        )
        return original_read(pattern, timeout)

    node._read_matching_response_locked = observe_state_before_ack

    result = node.stop_callback(None, response())

    assert result.success
    assert state_during_ack_wait == [(False, None, True)]
    assert node.live_follow_state_publisher.values == [True, False]


def test_stop_failure_disconnects_before_publishing_live_false():
    node, clock = arm_node([b'unexpected\r\n'])
    set_active(node, clock)
    serial_port = node.serial_port
    state_during_ack_wait = []
    original_read = node._read_matching_response_locked

    def observe_state_before_timeout(pattern, timeout):
        state_during_ack_wait.append(
            node.live_follow_state_publisher.values[-1]
        )
        return original_read(pattern, timeout)

    node._read_matching_response_locked = observe_state_before_timeout

    result = node.stop_callback(None, response())

    assert not result.success
    assert state_during_ack_wait == [True]
    assert serial_port.closed
    assert node.connection_state == 'DISCONNECTED'
    assert node.live_follow_state_publisher.values == [True, False]


def test_disable_invalidates_live_then_stops_before_disabling():
    node, clock = arm_node([
        b'Stopped ok\r\n',
        b'Disabled ok\r\n',
    ])
    set_active(node, clock)
    node.pending_live_target = (
        clock.nanoseconds,
        tuple(BASELINE_IDEAL),
    )
    public_state_during_responses = []
    original_read = node._read_matching_response_locked

    def observe_public_state(pattern, timeout):
        public_state_during_responses.append(
            node.live_follow_state_publisher.values[-1]
        )
        return original_read(pattern, timeout)

    node._read_matching_response_locked = observe_public_state

    result = node.disable_callback(None, response())

    assert result.success
    assert not node.live_follow_active
    assert node.pending_live_target is None
    assert node.serial_port.writes == [b'!STOP\n', b'!DISABLE\n']
    assert public_state_during_responses == [True, False]


def test_disconnect_invalidates_live_and_reconnect_does_not_rearm():
    node, clock = arm_node()
    set_active(node, clock)
    old_port = node.serial_port

    with node.serial_lock:
        node._disconnect_serial_locked()

    assert old_port.closed
    assert not node.live_follow_active
    assert node.pending_live_target is None
    assert not node.controller_started


def test_live_target_outside_configured_ideal_limit_is_rejected():
    node, clock = arm_node()
    set_active(node, clock, heartbeat_ready=False)
    joint_max = list(node.joint_max_rad)
    joint_max[1] = math.radians(79.0)
    node.joint_max_rad = tuple(joint_max)

    node.live_joint_command_callback(
        live_message(
            clock.nanoseconds,
            joint='joint2',
            position_rad=math.radians(80.0),
        )
    )

    assert node.pending_live_target is None
    assert node.live_last_heartbeat_monotonic is None


def test_normal_move_validation_rejects_configured_ideal_limit_violation():
    node = make_node()
    invalid_ideal = [0.0, math.radians(180.0), 0.0, 0.0, 0.0, 0.0]

    assert not node.validate_joint_command(invalid_ideal)


def test_folded_plus_75_ideal_maps_to_raw_minus_75_and_is_accepted():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    set_clock(node, 2_000_000_000)

    assert node.validate_joint_command(BASELINE_IDEAL)
    node.joint_command_callback(
        SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=2, nanosec=0),
            ),
            name=list(JOINT_NAMES),
            position=list(BASELINE_IDEAL),
        )
    )

    assert node.serial_port.writes[-1] == (
        b'&0.00,-75.00,180.00,0.00,0.00,0.00,30.00\n'
    )


def test_normal_callback_always_appends_explicit_speed():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    set_clock(node, 2_000_000_000)

    node.joint_command_callback(
        SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=2, nanosec=0),
            ),
            name=list(JOINT_NAMES),
            position=[0.0] * 6,
        )
    )

    assert node.serial_port.writes[-1].endswith(b',30.00\n')


def test_normal_and_live_routes_accept_nonzero_j6():
    normal = make_node([b'Started ok\r\n', b'15\r\n'])
    set_clock(normal, 2_000_000_000)
    positions = list(BASELINE_IDEAL)
    positions[5] = math.radians(1.0)
    normal.joint_command_callback(
        SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=2, nanosec=0),
            ),
            name=list(JOINT_NAMES),
            position=positions,
        )
    )
    assert any(write.startswith(b'&') for write in normal.serial_port.writes)

    live, clock = arm_node()
    set_active(live, clock, heartbeat_ready=False)
    live.live_joint_command_callback(
        full_live_message(clock.nanoseconds, positions)
    )
    assert live.pending_live_target is not None
    assert live.serial_port.writes == []


def test_normal_route_reorders_names_and_accepts_nonzero_j6():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    set_clock(node, 2_000_000_000)
    names = list(reversed(JOINT_NAMES))
    positions_by_name = dict(zip(JOINT_NAMES, BASELINE_IDEAL))
    positions_by_name['joint6'] = math.radians(1.0)

    node.joint_command_callback(
        SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=2, nanosec=0),
            ),
            name=names,
            position=[positions_by_name[name] for name in names],
        )
    )

    assert any(write.startswith(b'&') for write in node.serial_port.writes)


def test_final_serial_route_accepts_nonzero_raw_j6():
    node = make_node([b'Started ok\r\n', b'15\r\n'])

    assert node.send_joint_command('&1,2,3,4,5,0.01,30\n')
    assert any(write.startswith(b'&') for write in node.serial_port.writes)


def test_live_accepts_nonzero_j6_feedback():
    node, clock = arm_node([b'15\r\n'])
    set_active(node, clock)
    node.current_ideal_radians = list(node.current_ideal_radians)
    node.current_ideal_radians[5] = math.radians(1.0)
    node.pending_live_target = (
        clock.nanoseconds,
        tuple(BASELINE_IDEAL),
    )

    node.process_live_follow()

    assert node.live_follow_active
    assert any(write.startswith(b'&') for write in node.serial_port.writes)


@pytest.mark.parametrize(
    'attribute',
    [
        'poll_period',
        'live_command_period_sec',
        'live_command_max_age_sec',
        'live_feedback_timeout_sec',
        'live_watchdog_timeout_sec',
        'live_max_step_deg',
        'live_zero_delta_deg',
    ],
)
def test_live_configuration_rejects_nonfinite_values(attribute):
    node = make_node()
    setattr(node, attribute, math.inf)

    with pytest.raises(ValueError, match='finite'):
        node._validate_live_configuration()


@pytest.mark.parametrize(
    ('attribute', 'value'),
    [
        ('poll_period', 1.01),
        ('live_command_period_sec', 0.51),
        ('live_command_max_age_sec', 1.01),
        ('live_feedback_timeout_sec', 2.01),
        ('live_watchdog_timeout_sec', 1.01),
        ('live_max_step_deg', 5.01),
    ],
)
def test_live_configuration_rejects_excessive_values(attribute, value):
    node = make_node()
    setattr(node, attribute, value)

    with pytest.raises(ValueError):
        node._validate_live_configuration()


@pytest.mark.parametrize(
    ('attribute', 'value'),
    [
        ('live_command_max_age_sec', 0.05),
        ('live_feedback_timeout_sec', 0.19),
        ('live_watchdog_timeout_sec', 0.19),
        ('live_zero_delta_deg', -0.01),
        ('live_zero_delta_deg', 1.0),
    ],
)
def test_live_configuration_rejects_unsafe_relationships(attribute, value):
    node = make_node()
    setattr(node, attribute, value)

    with pytest.raises(ValueError):
        node._validate_live_configuration()


def test_live_configuration_rejects_age_longer_than_watchdog():
    node = make_node()
    node.live_command_max_age_sec = 0.31

    with pytest.raises(ValueError, match='must not exceed'):
        node._validate_live_configuration()


@pytest.mark.parametrize('speed', [math.inf, 0.0, 100.01])
def test_speed_parameter_rejects_nonfinite_or_out_of_range(speed):
    node = make_node()
    node.get_parameter = lambda _name: SimpleNamespace(value=speed)

    with pytest.raises(ValueError):
        node._validated_speed_parameter('normal_joint_speed')


@pytest.mark.parametrize('speed', [math.inf, 0.0, 20.01])
def test_live_speed_rejects_invalid_or_above_low_speed_cap(speed):
    node = make_node()
    node.get_parameter = lambda _name: SimpleNamespace(value=speed)

    with pytest.raises(ValueError):
        node._validated_speed_parameter(
            'live_joint_speed',
            maximum=driver_module.MAX_LIVE_JOINT_SPEED,
        )


def test_live_speed_must_not_exceed_normal_speed():
    node = make_node()
    node.normal_joint_speed = 5.0
    node.live_joint_speed = 10.0

    with pytest.raises(ValueError, match='normal_joint_speed'):
        node._validate_speed_configuration()


def test_live_speed_cap_is_allowed_when_normal_speed_is_not_lower():
    node = make_node()
    node.normal_joint_speed = 20.0
    node.live_joint_speed = driver_module.MAX_LIVE_JOINT_SPEED

    node._validate_speed_configuration()


def test_mock_live_state_machine_never_writes_serial():
    node, clock = arm_node()
    serial_port = FakeSerial()
    node.serial_port = serial_port
    node.use_mock = True
    node.mock_raw_degrees = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]

    armed = node.live_follow_callback(live_request(True), response())
    assert armed.success
    clock.nanoseconds += 1
    node.live_follow_heartbeat_callback(heartbeat(clock.nanoseconds))
    clock.nanoseconds += 1
    node.live_joint_command_callback(
        live_message(clock.nanoseconds, position_rad=math.radians(10.0))
    )
    node.process_live_follow()
    stopped = node.live_follow_callback(live_request(False), response())

    assert stopped.success
    assert serial_port.writes == []
    assert math.degrees(node.current_ideal_radians[0]) == 1.0
    assert not node.live_follow_active
    assert node.live_follow_state_publisher.values == [True, False]
