import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time as TimeMessage

from arm_driver.arm_command_node import ArmCommandNode
import arm_driver.arm_driver_node as driver_module
from test_start_handshake import FakeLogger


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _CompletedFuture:
    def done(self):
        return True

    def result(self):
        return SimpleNamespace(success=True, message='stopped')


class _StopClient:
    def __init__(self):
        self.wait_calls = 0
        self.call_count = 0

    def wait_for_service(self, timeout_sec):
        assert timeout_sec == 1.0
        self.wait_calls += 1
        return True

    def call_async(self, _request):
        self.call_count += 1
        return _CompletedFuture()


class _ForbiddenDisableClient:
    def wait_for_service(self, _timeout_sec):
        raise AssertionError('STOP must not wait on the DISABLE client')


@pytest.fixture
def construct_driver(monkeypatch):
    """Run the real constructor with fake ROS endpoints, never DDS or serial."""
    parameters = {}
    monkeypatch.setattr(driver_module.Node, '__init__', lambda *_a, **_k: None)
    monkeypatch.setattr(
        driver_module.Node, 'declare_parameter',
        lambda _self, name, default: parameters.setdefault(name, default),
    )
    monkeypatch.setattr(
        driver_module.Node, 'get_parameter',
        lambda _self, name: SimpleNamespace(value=parameters[name]),
    )
    monkeypatch.setattr(driver_module.Node, 'get_logger', lambda _: FakeLogger())
    monkeypatch.setattr(
        driver_module.Node, 'get_clock',
        lambda _: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_000_000_000, to_msg=TimeMessage,
        )),
    )
    monkeypatch.setattr(
        driver_module.Node, 'create_publisher',
        lambda *_a, **_k: SimpleNamespace(publish=lambda _msg: None),
    )
    for method in ('create_subscription', 'create_service', 'create_timer'):
        monkeypatch.setattr(driver_module.Node, method, lambda *_a, **_k: None)

    def forbidden_serial(*_args, **_kwargs):
        raise AssertionError('offline regression must never construct serial')

    monkeypatch.setattr(driver_module.serial, 'Serial', forbidden_serial)

    def construct(**overrides):
        parameters.update(overrides)
        return driver_module.ArmDriveNode()

    return construct


def test_default_constructor_and_callbacks_never_access_serial(construct_driver):
    node = construct_driver()
    assert node.use_mock is True
    assert node.allow_motion_commands is False
    node.publish_joint_state()
    node.joint_command_callback(SimpleNamespace())
    node.live_joint_command_callback(SimpleNamespace())
    assert not node.send_joint_command('&0,-75,180,0,0,0,30\n')
    assert not node._start_controller_locked()
    assert not node.live_follow_callback(
        SimpleNamespace(data=True), SimpleNamespace(success=False, message=''),
    ).success
    for callback in (node.stop_callback, node.disable_callback):
        callback(None, SimpleNamespace(success=False, message=''))
    node.close_serial_port()
    assert node.serial_port is None
    assert node.controller_started is False


def test_motion_permission_alone_stays_mock(construct_driver):
    node = construct_driver(allow_motion_commands=True)
    node.publish_joint_state()
    assert node.use_mock is True
    assert node.send_joint_command('&0,-75,180,0,0,0,30\n')
    node.close_serial_port()
    assert node.serial_port is None


@pytest.mark.parametrize('authority', [False, True])
def test_real_mode_requires_separate_motion_opt_in(
    construct_driver, monkeypatch, authority,
):
    connections = []
    monkeypatch.setattr(
        driver_module.ArmDriveNode, '_maybe_reconnect_locked',
        lambda node: connections.append(node.use_mock),
    )
    overrides = {'use_mock': False}
    if authority:
        overrides['allow_motion_commands'] = True
    node = construct_driver(**overrides)
    assert connections == [False]
    assert node.allow_motion_commands is authority
    if not authority:
        assert not node._start_controller_locked()
        assert not node.send_joint_command('&0,-75,180,0,0,0,30\n')
    # Exact opted-in START/MOVE bytes and ACKs are covered in test_start_handshake.


def test_stop_command_waits_on_stop_client():
    stop_client = _StopClient()
    node = SimpleNamespace(
        stop_client=stop_client,
        disable_client=_ForbiddenDisableClient(),
        get_logger=lambda: _Logger(),
    )

    ArmCommandNode.stop_arm(node)

    assert stop_client.wait_calls == 1
    assert stop_client.call_count == 1


def test_direct_command_cli_publishes_nonzero_j6():
    published = []
    node = SimpleNamespace(
        get_logger=lambda: _Logger(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=TimeMessage)
        ),
        joint_names=[f'joint{index}' for index in range(1, 7)],
        command_publisher=SimpleNamespace(
            publish=published.append
        ),
    )

    assert ArmCommandNode.send_joint_command(
        node, [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    )
    assert published[0].position[5] == pytest.approx(math.radians(1.0))
