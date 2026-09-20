import threading
import time
from types import SimpleNamespace

import arm_driver.arm_driver_node as driver_module
import pytest
from arm_driver.arm_driver_node import ArmDriveNode


class FakeSerial:
    def __init__(self, responses=(), write_error=None):
        self.is_open = True
        self.responses = list(responses)
        self.write_error = write_error
        self.writes = []
        self.timeout = 0.001
        self.closed = False

    @property
    def in_waiting(self):
        if not self.responses:
            return 0
        return len(self.responses[0])

    def write(self, data):
        if self.write_error is not None:
            raise self.write_error
        self.writes.append(data)
        return len(data)

    def read(self, _size):
        if self.responses:
            return self.responses.pop(0)
        return b''

    def close(self):
        self.closed = True
        self.is_open = False


class FakeLogger:
    def debug(self, _message):
        pass

    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def make_node(responses=(), write_error=None):
    node = ArmDriveNode.__new__(ArmDriveNode)
    node.use_mock = False
    node.allow_motion_commands = True
    node.serial_port_name = '/dev/ttyACM0'
    node.active_serial_port_name = '/dev/ttyACM0'
    node.preferred_serial_port_name = None
    node.baud_rate = 115200
    node.poll_period = 0.1
    node.serial_port = FakeSerial(responses, write_error)
    node.serial_lock = threading.Lock()
    node.serial_rx_buffer = bytearray()
    node.controller_started = False
    node.current_ideal_radians = [0.0] * 6
    node.last_feedback_monotonic = time.monotonic()
    node.start_ack_timeout_sec = 0.01
    node.start_settle_delay_sec = 0.0
    node.command_ack_timeout_sec = 0.01
    node.joint_response_timeout_sec = 0.01
    node.disconnect_after_failures = 3
    node.consecutive_poll_failures = 0
    node.connection_state = 'CONNECTED'
    node.minimum_command_stamp_ns = None
    node.reconnect_initial_delay_sec = 0.1
    node.reconnect_max_delay_sec = 1.0
    node.reconnect_delay_sec = 0.1
    node.next_reconnect_time = 0.0
    node.normal_joint_speed = 30.0
    node.live_joint_speed = 10.0
    node.joint_min_rad = (-3.14,) * 6
    node.joint_max_rad = (3.14,) * 6
    node.live_command_period_sec = 0.1
    node.live_command_max_age_sec = 0.25
    node.live_feedback_timeout_sec = 0.35
    node.live_watchdog_timeout_sec = 0.30
    node.live_max_step_deg = 1.0
    node.live_zero_delta_deg = 0.02
    node.live_follow_active = False
    node.live_follow_tracking = False
    node.live_epoch_stamp_ns = None
    node.live_armed_monotonic = None
    node.live_last_message_stamp_ns = None
    node.live_last_heartbeat_stamp_ns = None
    node.live_last_heartbeat_monotonic = None
    node.live_last_send_monotonic = None
    node.live_last_send_feedback_monotonic = None
    node.live_last_motion_command = None
    node.pending_live_target = None
    node._last_live_state_published = None
    node.joint_names = [f'joint{index}' for index in range(1, 7)]
    node.get_logger = lambda: FakeLogger()
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=time.time_ns())
    )
    return node


def make_joint_command_message(sec, nanosec):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        name=[f'joint{index}' for index in range(1, 7)],
        position=[0.0] * 6,
    )


def test_move_waits_for_start_and_queue_ack_without_flush():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    command = '&0.00,-75.00,180.00,0.00,0.00,0.00\n'

    assert node.send_joint_command(command)
    assert node.serial_port.writes == [
        b'!START\n',
        command.encode('ascii'),
    ]
    assert node.controller_started


def test_first_move_waits_for_controller_start_to_settle(monkeypatch):
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    node.start_settle_delay_sec = 0.5
    events = []
    original_write = node.serial_port.write

    def record_write(data):
        events.append(('write', data))
        return original_write(data)

    node.serial_port.write = record_write
    monkeypatch.setattr(
        driver_module.time,
        'sleep',
        lambda seconds: events.append(('sleep', seconds)),
    )

    command = '&0.00,-75.00,180.00,0.00,0.00,0.00,30.00\n'
    assert node.send_joint_command(command)
    assert events == [
        ('write', b'!START\n'),
        ('sleep', 0.5),
        ('write', command.encode('ascii')),
    ]


def test_already_started_controller_does_not_delay_move(monkeypatch):
    node = make_node([b'15\r\n'])
    node.controller_started = True
    node.start_settle_delay_sec = 0.5
    sleeps = []
    monkeypatch.setattr(driver_module.time, 'sleep', sleeps.append)

    assert node.send_joint_command('&1,2,3,4,5,6,30\n')
    assert sleeps == []


def test_start_handshake_alone_does_not_apply_move_settle_delay(monkeypatch):
    node = make_node([b'Started ok\r\n'])
    node.start_settle_delay_sec = 0.5
    sleeps = []
    monkeypatch.setattr(driver_module.time, 'sleep', sleeps.append)

    assert node._start_controller_locked()
    assert sleeps == []


def test_motion_interlock_blocks_move_and_automatic_start():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    node.allow_motion_commands = False

    assert not node.send_joint_command('&1,2,3,4,5,0,30\n')
    assert not node._start_controller_locked()
    assert node.serial_port.writes == []
    assert not node.controller_started


def test_motion_interlock_still_allows_feedback_stop_and_disable():
    node = make_node([
        b'ok 0 -75 180 0 0 0\r\n',
        b'Stopped ok\r\n',
        b'Disabled ok\r\n',
    ])
    node.allow_motion_commands = False

    with node.serial_lock:
        assert node._query_joint_positions_locked() == [
            0.0, -75.0, 180.0, 0.0, 0.0, 0.0,
        ]
    assert node.stop_callback(
        None, SimpleNamespace(success=False, message='')
    ).success
    assert node.disable_callback(
        None, SimpleNamespace(success=False, message='')
    ).success
    assert node.serial_port.writes == [
        b'#GETJPOS\n',
        b'!STOP\n',
        b'!DISABLE\n',
    ]


def test_move_rejects_stale_feedback_before_start_or_enqueue():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    node.last_feedback_monotonic = (
        time.monotonic() - node.live_feedback_timeout_sec - 0.1
    )

    assert not node.send_joint_command('&1,2,3,4,5,0,30\n')
    assert node.serial_port.writes == []
    assert node.serial_port.is_open
    assert node.connection_state == 'CONNECTED'


def test_move_rechecks_feedback_freshness_after_taking_serial_lock():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    freshness = iter((True, False))
    node._feedback_is_fresh = lambda: next(freshness)

    assert not node.send_joint_command('&1,2,3,4,5,0,30\n')
    assert node.serial_port.writes == []
    assert node.serial_port.is_open


def test_zero_remaining_queue_slots_is_a_valid_accepted_move():
    node = make_node([b'Started ok\r\n', b'0\r\n'])

    assert node.send_joint_command('&1,2,3,4,5,0\n')


def test_reconnect_generation_rejects_old_command_but_accepts_new_one():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    node.minimum_command_stamp_ns = 2_000_000_100

    node.joint_command_callback(make_joint_command_message(2, 99))
    assert node.serial_port.writes == []

    node.joint_command_callback(make_joint_command_message(2, 101))
    assert node.serial_port.writes[0] == b'!START\n'
    assert node.serial_port.writes[1].startswith(b'&')


def test_zero_stamp_command_is_rejected_before_start_or_move():
    node = make_node([b'Started ok\r\n', b'15\r\n'])

    node.joint_command_callback(make_joint_command_message(0, 0))

    assert node.serial_port.writes == []


def test_send_rechecks_generation_stamp_under_serial_lock():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    node.minimum_command_stamp_ns = 3_000_000_100
    stale_stamp = SimpleNamespace(sec=3, nanosec=99)

    assert not node.send_joint_command(
        '&1,2,3,4,5,0\n',
        command_stamp=stale_stamp,
    )
    assert node.serial_port.writes == []


def test_command_at_exact_stamp_barrier_is_rejected_fail_closed():
    node = make_node([b'Started ok\r\n', b'15\r\n'])
    node.minimum_command_stamp_ns = 3_000_000_100
    equal_stamp = SimpleNamespace(sec=3, nanosec=100)

    assert not node.send_joint_command(
        '&1,2,3,4,5,0\n',
        command_stamp=equal_stamp,
    )
    assert node.serial_port.writes == []


def test_command_stamp_barrier_never_regresses_when_ros_clock_moves_back():
    node = make_node()
    node.minimum_command_stamp_ns = 5_000_000_000
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=4_000_000_000)
    )

    node._advance_command_stamp_barrier()

    assert node.minimum_command_stamp_ns == 5_000_000_000


def test_reconnect_epoch_is_recorded_only_on_first_probe_success():
    node = make_node()
    clock_time = SimpleNamespace(nanoseconds=5_000_000_123)
    node.get_clock = lambda: SimpleNamespace(now=lambda: clock_time)
    node.connection_state = 'PROBING'

    with node.serial_lock:
        node._record_poll_success_locked()

    assert node.minimum_command_stamp_ns == 5_000_000_123

    clock_time.nanoseconds = 9_000_000_456
    with node.serial_lock:
        node._record_poll_success_locked()

    assert node.minimum_command_stamp_ns == 5_000_000_123


def test_reconnect_epoch_does_not_regress_after_ros_clock_rollback():
    node = make_node()
    node.minimum_command_stamp_ns = 5_000_000_123
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=4_000_000_000)
    )
    node.connection_state = 'PROBING'

    with node.serial_lock:
        node._record_poll_success_locked()

    assert node.minimum_command_stamp_ns == 5_000_000_123


def test_move_is_not_written_when_start_ack_times_out():
    node = make_node([b'unexpected response\n'])
    old_port = node.serial_port

    assert not node.send_joint_command('&1,2,3,4,5,0\n')
    assert old_port.writes == [b'!START\n']
    assert old_port.closed
    assert node.serial_port is None
    assert node.connection_state == 'DISCONNECTED'
    assert node.current_ideal_radians is None
    assert not node.controller_started


def test_delayed_move_ok_is_not_mistaken_for_joint_state():
    node = make_node([
        b'Started ok\r\n',
        b'15\r\n',
        b'ok\r\n',
        b'ok 1 2 3 4 5 6\r\n',
    ])

    assert node.send_joint_command('&1,2,3,4,5,0\n')
    with node.serial_lock:
        joints = node._query_joint_positions_locked()

    assert joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert node.serial_port.writes[-1] == b'#GETJPOS\n'


def test_glued_queue_and_completion_ack_are_classified():
    node = make_node([
        b'Started ok\r\n',
        b'15ok\r\n',
        b'ok -1 -2 -3 -4 -5 -6\r\n',
    ])

    assert node.send_joint_command('&1,2,3,4,5,0\n')
    with node.serial_lock:
        joints = node._query_joint_positions_locked()

    assert joints == [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0]


def test_completion_ack_before_glued_queue_ack_is_classified():
    node = make_node([b'Started ok\r\n', b'ok15\r\n\r\n'])

    assert node.send_joint_command('&1,2,3,4,5,0\n')


def test_stale_joint_response_is_not_mistaken_for_queue_ack():
    node = make_node([
        b'Started ok\r\n',
        b'ok 1 2 3 4 5 6\r\n',
        b'15\r\n',
    ])

    assert node.send_joint_command('&1,2,3,4,5,0\n')


def test_sixteen_free_slots_is_a_valid_raced_queue_ack():
    node = make_node([b'Started ok\r\n', b'16\r\n'])

    assert node.send_joint_command('&1,2,3,4,5,0\n')


@pytest.mark.parametrize('invalid_ack', [b'254\r\n', b'255\r\n'])
def test_move_rejects_invalid_or_failed_queue_ack(invalid_ack):
    node = make_node([b'Started ok\r\n', invalid_ack])
    old_port = node.serial_port

    assert not node.send_joint_command('&1,2,3,4,5,0\n')
    assert old_port.writes == [
        b'!START\n',
        b'&1,2,3,4,5,0\n',
    ]
    assert old_port.closed
    assert node.serial_port is None
    assert node.current_ideal_radians is None


def test_fragmented_queue_ack_waits_for_its_frame_terminator():
    node = make_node([b'Started ok\r\n', b'1', b'5\r\n'])

    assert node.send_joint_command('&1,2,3,4,5,0\n')


def test_fragmented_joint_response_is_reassembled():
    node = make_node([
        b'ok 1.25 -2.',
        b'5 3 4 5 6\r\n',
    ])

    with node.serial_lock:
        joints = node._query_joint_positions_locked()

    assert joints == [1.25, -2.5, 3.0, 4.0, 5.0, 6.0]


def test_joint_response_waits_for_final_number_frame_terminator():
    node = make_node([
        b'ok 1 2 3 4 5 6',
        b'0\r\n',
    ])

    with node.serial_lock:
        joints = node._query_joint_positions_locked()

    assert joints == [1.0, 2.0, 3.0, 4.0, 5.0, 60.0]


def test_joint_response_accepts_interleaved_completion_ack_at_end():
    node = make_node([b'ok 1 2 3 4 5 6ok\r\n\r\n'])

    with node.serial_lock:
        joints = node._query_joint_positions_locked()

    assert joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_stop_requires_ack_and_next_move_requires_new_start():
    node = make_node([
        b'ok\r\nStopped ok\r\n',
        b'Started ok\r\n',
        b'14\r\n',
    ])
    node.controller_started = True
    response = SimpleNamespace(success=False, message='')

    assert node.stop_callback(None, response).success
    assert not node.controller_started

    command = '&10.00,-75.00,180.00,0.00,0.00,0.00\n'
    assert node.send_joint_command(command)
    assert node.serial_port.writes == [
        b'!STOP\n',
        b'!START\n',
        command.encode('ascii'),
    ]
    assert node.controller_started


def test_stop_ack_accepts_interleaved_completion_ack_at_end():
    node = make_node([b'Stopped okok\r\n\r\n'])
    node.controller_started = True
    response = SimpleNamespace(success=False, message='')

    assert node.stop_callback(None, response).success
    assert not node.controller_started


def test_stop_barrier_rejects_older_move_but_allows_newer_stamp():
    node = make_node([
        b'Stopped ok\r\n',
        b'Started ok\r\n',
        b'15\r\n',
    ])
    barrier_ns = 2_000_000_000
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=barrier_ns)
    )
    response = SimpleNamespace(success=False, message='')

    assert node.stop_callback(None, response).success
    assert node.minimum_command_stamp_ns == barrier_ns

    old_stamp = SimpleNamespace(sec=1, nanosec=999_999_999)
    assert not node.send_joint_command(
        '&1,2,3,4,5,0,30\n',
        command_stamp=old_stamp,
    )
    assert node.serial_port.writes == [b'!STOP\n']

    new_stamp = SimpleNamespace(sec=2, nanosec=1)
    assert node.send_joint_command(
        '&1,2,3,4,5,0,30\n',
        command_stamp=new_stamp,
    )
    assert node.serial_port.writes == [
        b'!STOP\n',
        b'!START\n',
        b'&1,2,3,4,5,0,30\n',
    ]


def test_disable_advances_move_stamp_barrier_before_firmware_action():
    node = make_node([b'Disabled ok\r\n'])
    barrier_ns = 3_000_000_000
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=barrier_ns)
    )
    response = SimpleNamespace(success=False, message='')

    assert node.disable_callback(None, response).success

    assert node.minimum_command_stamp_ns == barrier_ns
    old_stamp = SimpleNamespace(sec=2, nanosec=999_999_999)
    assert not node.send_joint_command(
        '&1,2,3,4,5,0,30\n',
        command_stamp=old_stamp,
    )
    assert node.serial_port.writes == [b'!DISABLE\n']


def test_disable_reports_failure_without_confirmation():
    node = make_node([b'ok\r\n'])
    node.controller_started = True
    old_port = node.serial_port
    response = SimpleNamespace(success=True, message='')

    result = node.disable_callback(None, response)

    assert not result.success
    assert 'acknowledgement' in result.message.lower()
    assert not node.controller_started
    assert old_port.closed
    assert node.serial_port is None
    assert node.connection_state == 'DISCONNECTED'
    assert node.current_ideal_radians is None


def test_stop_timeout_also_disconnects_unsynchronized_protocol():
    node = make_node([b'ok\r\n'])
    node.controller_started = True
    old_port = node.serial_port
    response = SimpleNamespace(success=True, message='')

    result = node.stop_callback(None, response)

    assert not result.success
    assert old_port.closed
    assert node.serial_port is None
    assert node.connection_state == 'DISCONNECTED'
    assert node.current_ideal_radians is None


def test_late_move_ack_cannot_be_reused_after_timeout():
    node = make_node([b'Started ok\r\n'])
    old_port = node.serial_port

    assert not node.send_joint_command('&1,2,3,4,5,0\n')
    old_port.responses.append(b'15\r\n')

    assert not node.send_joint_command('&6,5,4,3,2,0\n')
    assert old_port.writes == [
        b'!START\n',
        b'&1,2,3,4,5,0\n',
    ]
    assert old_port.closed


def test_write_timeout_disconnects_without_replaying_move():
    error = driver_module.serial.SerialTimeoutException('write timed out')
    node = make_node(write_error=error)
    command = '&1,2,3,4,5,0\n'

    assert not node.send_joint_command(command)
    assert node.serial_port is None
    assert node.connection_state == 'DISCONNECTED'
    assert node.current_ideal_radians is None


def test_shutdown_continues_when_second_interrupt_arrives(monkeypatch):
    calls = []

    class FakeNode:
        def close_serial_port(self):
            calls.append('close')

        def destroy_node(self):
            calls.append('destroy')
            raise KeyboardInterrupt

    monkeypatch.setattr(
        driver_module.rclpy,
        'try_shutdown',
        lambda: calls.append('shutdown'),
    )

    driver_module._shutdown_node(FakeNode())

    assert calls == ['close', 'destroy', 'shutdown']
