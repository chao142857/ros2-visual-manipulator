import threading
import time

import arm_driver.arm_driver_node as driver_module
from arm_driver.arm_driver_node import ArmDriveNode

from test_start_handshake import FakeLogger
from test_start_handshake import FakeSerial


def make_disconnected_node():
    node = ArmDriveNode.__new__(ArmDriveNode)
    node.use_mock = False
    node.allow_motion_commands = True
    node.serial_port_name = '/dev/ttyACM0'
    node.active_serial_port_name = None
    node.preferred_serial_port_name = None
    node.baud_rate = 115200
    node.poll_period = 0.1
    node.serial_read_timeout_sec = 0.05
    node.serial_write_timeout_sec = 0.2
    node.joint_response_timeout_sec = 0.01
    node.serial_port = None
    node.serial_lock = threading.Lock()
    node.serial_rx_buffer = bytearray(b'stale')
    node.controller_started = True
    node.current_ideal_radians = [1.0] * 6
    node.last_feedback_monotonic = time.monotonic()
    node.disconnect_after_failures = 3
    node.consecutive_poll_failures = 0
    node.connection_state = 'DISCONNECTED'
    node.minimum_command_stamp_ns = None
    node.reconnect_initial_delay_sec = 0.1
    node.reconnect_max_delay_sec = 1.0
    node.reconnect_delay_sec = 0.1
    node.next_reconnect_time = 0.0
    node.joint_min_rad = (-3.14,) * 6
    node.joint_max_rad = (3.14,) * 6
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
    node.get_logger = lambda: FakeLogger()
    return node


def test_three_poll_failures_clear_state_and_close_port():
    node = make_disconnected_node()
    old_port = FakeSerial()
    node.serial_port = old_port
    node.connection_state = 'CONNECTED'

    with node.serial_lock:
        node._record_poll_failure_locked('one')
        node._record_poll_failure_locked('two')

    assert node.serial_port is old_port
    assert node.current_ideal_radians == [1.0] * 6

    with node.serial_lock:
        node._record_poll_failure_locked('three')

    assert old_port.closed
    assert node.serial_port is None
    assert node.current_ideal_radians is None
    assert not node.controller_started
    assert node.connection_state == 'DISCONNECTED'


def test_reconnect_opens_only_and_never_starts_or_replays(monkeypatch):
    node = make_disconnected_node()
    new_port = FakeSerial([b'ok 1 2 3 4 5 6\r\n'])
    opened_with = {}

    def fake_serial(**kwargs):
        opened_with.update(kwargs)
        return new_port

    monkeypatch.setattr(driver_module.serial, 'Serial', fake_serial)
    monkeypatch.setattr(
        driver_module,
        'resolve_stable_serial_port',
        lambda port: '/dev/serial/by-id/arm-controller',
    )

    with node.serial_lock:
        assert node._maybe_reconnect_locked(now=10.0)

    assert new_port.writes == []
    assert not node.controller_started
    assert node.current_ideal_radians is None
    assert node.serial_rx_buffer == bytearray()
    assert opened_with['port'] == '/dev/serial/by-id/arm-controller'
    assert opened_with['write_timeout'] == 0.2
    assert opened_with['exclusive'] is True
    assert not node.send_joint_command('&1,2,3,4,5,0\n')
    assert new_port.writes == []

    with node.serial_lock:
        joints = node._query_joint_positions_locked()

    assert joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert new_port.writes == [b'#GETJPOS\n']
    assert not node.controller_started


def test_reconnect_respects_backoff(monkeypatch):
    node = make_disconnected_node()
    node.next_reconnect_time = 20.0
    calls = []
    monkeypatch.setattr(
        driver_module.serial,
        'Serial',
        lambda **kwargs: calls.append(kwargs),
    )

    with node.serial_lock:
        assert not node._maybe_reconnect_locked(now=19.9)

    assert calls == []


def test_reconnect_reuses_discovered_by_id_after_reenumeration(monkeypatch):
    node = make_disconnected_node()
    ports = [FakeSerial(), FakeSerial()]
    opened_names = []

    def fake_serial(**kwargs):
        opened_names.append(kwargs['port'])
        return ports.pop(0)

    monkeypatch.setattr(driver_module.serial, 'Serial', fake_serial)
    monkeypatch.setattr(
        driver_module,
        'resolve_stable_serial_port',
        lambda _port: '/dev/serial/by-id/arm-controller',
    )

    with node.serial_lock:
        assert node._maybe_reconnect_locked(now=10.0)
        node._disconnect_serial_locked()
        reconnect_at = node.next_reconnect_time

    monkeypatch.setattr(
        driver_module,
        'resolve_stable_serial_port',
        lambda _port: '/dev/ttyACM1',
    )

    with node.serial_lock:
        assert node._maybe_reconnect_locked(now=reconnect_at + 0.01)

    assert opened_names == [
        '/dev/serial/by-id/arm-controller',
        '/dev/serial/by-id/arm-controller',
    ]
    assert node.preferred_serial_port_name == (
        '/dev/serial/by-id/arm-controller'
    )
    assert node.serial_port.writes == []
