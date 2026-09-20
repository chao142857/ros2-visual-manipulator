from pathlib import Path
from types import SimpleNamespace

import pytest

import gripper_driver.gripper_driver_node as driver_module
from gripper_driver.gripper_driver_node import (
    GripperDriverNode,
    validate_gripper_configuration,
)


class _Logger:
    def __init__(self):
        self.messages = []
        self.errors = []
        self.infos = []

    def warning(self, message):
        self.messages.append(message)

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)


@pytest.fixture
def driver_environment(monkeypatch):
    """Run the real constructor with fake ROS methods and forbidden serial."""
    parameters = {}
    subscriptions = []
    logger = _Logger()

    def forbidden_serial(*_args, **_kwargs):
        pytest.fail('disabled driver must not construct a serial transport')

    def create_subscription(_self, *arguments):
        subscriptions.append(arguments)
        return SimpleNamespace(topic=arguments[1])

    monkeypatch.setattr(driver_module.Node, '__init__', lambda *_args: None)
    monkeypatch.setattr(
        driver_module.Node, 'declare_parameter',
        lambda _self, name, default: parameters.setdefault(name, default),
    )
    monkeypatch.setattr(
        driver_module.Node, 'get_parameter',
        lambda _self, name: SimpleNamespace(value=parameters[name]),
    )
    monkeypatch.setattr(driver_module.Node, 'get_logger', lambda _self: logger)
    monkeypatch.setattr(
        driver_module.Node, 'create_subscription', create_subscription,
    )
    monkeypatch.setattr(driver_module.serial, 'Serial', forbidden_serial)
    monkeypatch.setattr(driver_module.time, 'sleep', lambda _duration: None)
    return SimpleNamespace(
        parameters=parameters, subscriptions=subscriptions, logger=logger,
    )


class _FakeSerial:
    def __init__(self):
        self.writes = []
        self.responses = []
        self.open_count = 0
        self.flush_count = 0
        self.closed = False

    def open(self):
        self.open_count += 1

    def write(self, data):
        self.writes.append(data)
        if data == b'PING\n':
            self.responses.append(b'PONG ESP32_PCA9685_GRIPPER_V1\n')
        elif data.startswith(b'SET '):
            self.responses.append(b'OK ' + data)
        return len(data)

    def read(self, _size):
        return self.responses.pop(0) if self.responses else b''

    def flush(self):
        self.flush_count += 1

    def close(self):
        self.closed = True


def test_default_constructor_is_disabled_without_serial_or_command_endpoint(
    driver_environment,
):
    node = GripperDriverNode()

    assert node.allow_hardware_actuation is False
    assert node.serial is None
    assert node.pulse_subscription is None
    assert node.controller_connected is False
    assert driver_environment.subscriptions == []
    assert driver_environment.logger.infos == []
    assert any(
        'hardware disabled' in message
        for message in driver_environment.logger.messages
    )


def test_disabled_callback_rejects_valid_pulse_before_send_or_read(
    driver_environment, monkeypatch,
):
    node = GripperDriverNode()

    def forbidden_io(*_args):
        pytest.fail('disabled callback must not send or read serial')

    monkeypatch.setattr(node, 'send_gripper_command', forbidden_io)
    monkeypatch.setattr(node, 'read_serial_reply', forbidden_io)
    node.pulse_callback(SimpleNamespace(data=1200))

    assert driver_environment.logger.infos == []
    assert 'Rejected gripper pulse' in driver_environment.logger.messages[-1]


@pytest.mark.parametrize('command', ['PING', 'SET 0 1200 600'])
def test_disabled_low_level_send_never_writes_even_with_transport_present(
    driver_environment, command,
):
    node = GripperDriverNode()
    transport = _FakeSerial()
    node.serial = transport

    assert node.send_gripper_command(command) is False
    assert transport.writes == []
    assert transport.flush_count == 0
    assert transport.open_count == 0
    assert driver_environment.logger.infos == []


def test_explicit_hardware_opt_in_preserves_handshake_and_set_bytes(
    driver_environment, monkeypatch,
):
    driver_environment.parameters['allow_hardware_actuation'] = True
    transport = _FakeSerial()
    monkeypatch.setattr(driver_module.serial, 'Serial', lambda: transport)

    node = GripperDriverNode()

    assert node.controller_connected
    assert transport.open_count == 1
    assert transport.writes == [b'PING\n']
    assert node.pulse_subscription.topic == '/gripper/pulse_us'
    assert (node.open_pulse_us, node.close_pulse_us) == (600, 1300)
    assert transport.baudrate == 115200
    assert transport.dtr is False
    assert transport.rts is False

    node.pulse_callback(SimpleNamespace(data=1200))

    assert transport.writes == [b'PING\n', b'SET 0 1200 600\n']
    assert transport.flush_count == 2
    assert driver_environment.logger.errors == []
    assert "[RAW REPLY] 'OK SET 0 1200 600\\n'" in (
        driver_environment.logger.infos
    )


def test_gripper_configuration_accepts_any_in_range_pulses():
    assert validate_gripper_configuration(
        900, 1400
    ) == (900, 1400)
    assert validate_gripper_configuration(
        600, 1300
    ) == (600, 1300)
    assert validate_gripper_configuration(1200, 1200) == (1200, 1200)
    with pytest.raises(ValueError, match='between 600 and 1700'):
        validate_gripper_configuration(599, 1300)
    with pytest.raises(ValueError, match='between 600 and 1700'):
        validate_gripper_configuration(600, 1701)


def _enabled_node(logger, send_result=True, reply=''):
    return SimpleNamespace(
        allow_hardware_actuation=True,
        open_pulse_us=900,
        close_pulse_us=1400,
        gripper_channel=0,
        move_duration_ms=600,
        get_logger=lambda: logger,
        send_gripper_command=lambda _command: send_result,
        read_serial_reply=lambda _expected, _timeout: reply,
    )


def test_set_send_failure_returns_before_waiting_for_reply():
    logger = _Logger()
    node = _enabled_node(logger, send_result=False)
    node.read_serial_reply = lambda *_args: pytest.fail(
        'failed serial write must not wait for an acknowledgement'
    )

    GripperDriverNode.pulse_callback(node, SimpleNamespace(data=1200))

    assert logger.errors == []
    assert logger.infos == []


def test_open_does_not_require_joint_feedback():
    logger = _Logger()
    node = _enabled_node(logger, reply='OK SET 0 900 600')
    GripperDriverNode.pulse_callback(node, SimpleNamespace(data=900))
    assert logger.messages == []
    assert logger.errors == []


def test_unacknowledged_set_reply_is_reported_as_error():
    logger = _Logger()
    node = _enabled_node(logger, reply='ERR BAD COMMAND')

    GripperDriverNode.pulse_callback(node, SimpleNamespace(data=1200))

    assert len(logger.errors) == 1
    assert 'not acknowledged' in logger.errors[0]
    assert logger.infos == []


def test_ping_failure_does_not_leave_a_false_online_node():
    source = (
        Path(__file__).parents[1]
        / 'gripper_driver'
        / 'gripper_driver_node.py'
    ).read_text()
    assert "if not self.send_gripper_command('PING'):" in source
    assert "raise RuntimeError('Failed to send gripper PING')" in source
    expected = "raise RuntimeError('Gripper controller handshake failed')"
    assert expected in source
