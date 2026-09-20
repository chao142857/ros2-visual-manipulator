import math
import time

import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32

GRIPPER_MIN_PULSE_US = 600
GRIPPER_MAX_PULSE_US = 1700


def validate_gripper_configuration(
    open_pulse_us,
    close_pulse_us,
):
    """Return endpoint pulses when both are inside the allowed range."""

    try:
        values = (float(open_pulse_us), float(close_pulse_us))
    except (TypeError, ValueError) as error:
        raise ValueError(
            'gripper pulses must be positive finite integers'
        ) from error
    if not all(
        math.isfinite(value) and value.is_integer() and value > 0.0
        for value in values
    ):
        raise ValueError(
            'gripper pulses must be positive finite integers'
        )

    open_value, close_value = (int(value) for value in values)
    if not all(
        GRIPPER_MIN_PULSE_US <= value <= GRIPPER_MAX_PULSE_US
        for value in (open_value, close_value)
    ):
        raise ValueError('gripper pulses must be between 600 and 1700 us')
    return open_value, close_value


class GripperDriverNode(Node):

    def __init__(self):
        super().__init__('gripper_driver_node')

        self.controller_connected = False
        self.serial = None
        self.pulse_subscription = None
        self.declare_parameter('allow_hardware_actuation', False)
        self.allow_hardware_actuation = self.get_parameter(
            'allow_hardware_actuation'
        ).value
        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/CONFIGURE_GRIPPER_PORT'
        )

        self.declare_parameter(
            'baud_rate',
            115200
        )

        self.declare_parameter(
            'gripper_channel',
            0
        )

        self.declare_parameter(
            'open_pulse_us',
            600
        )

        self.declare_parameter(
            'close_pulse_us',
            1300
        )

        self.declare_parameter(
            'move_duration_ms',
            600
        )

        serial_port = self.get_parameter(
            'serial_port'
        ).value

        self.gripper_channel = self.get_parameter(
            'gripper_channel'
        ).value

        self.open_pulse_us = self.get_parameter(
            'open_pulse_us'
        ).value

        self.close_pulse_us = self.get_parameter(
            'close_pulse_us'
        ).value

        validated_pulses = validate_gripper_configuration(
            self.open_pulse_us,
            self.close_pulse_us,
        )
        self.open_pulse_us, self.close_pulse_us = validated_pulses

        self.move_duration_ms = self.get_parameter(
            'move_duration_ms'
        ).value
        baud_rate = self.get_parameter(
            'baud_rate'
        ).value

        if not self.allow_hardware_actuation:
            self.get_logger().warning(
                'Gripper hardware disabled: allow_hardware_actuation=false; '
                'serial remains closed and command subscription is disabled. '
                'This is not a mock gripper or a successful hardware connection.'
            )
            return

        self.serial = serial.Serial()

        self.serial.port = serial_port
        self.serial.baudrate = baud_rate
        self.serial.bytesize = 8
        self.serial.parity = serial.PARITY_NONE
        self.serial.stopbits = 1
        self.serial.timeout = 0.05

        self.serial.dtr = False
        self.serial.rts = False

        self.serial.open()

        time.sleep(1.0)

        if not self.send_gripper_command('PING'):
            self.serial.close()
            raise RuntimeError('Failed to send gripper PING')

        deadline = time.monotonic() + 1.5
        received = bytearray()

        while time.monotonic() < deadline:

            received.extend(
                self.serial.read(256)
            )

            text = received.decode(
                'utf-8',
                errors='replace'
            )

            if 'PONG ESP32_PCA9685_GRIPPER_V1' in text:
                self.controller_connected = True

                self.get_logger().info(
                    'Gripper controller connected'
                )
                break

        if not self.controller_connected:
            self.get_logger().error(
                f'Gripper PING failed: {text!r}'
            )
            self.serial.close()
            raise RuntimeError('Gripper controller handshake failed')

        self.pulse_subscription = self.create_subscription(
            Int32,
            '/gripper/pulse_us',
            self.pulse_callback,
            10
        )
        self.get_logger().info(
            'Gripper Driver Node started'
        )

    def pulse_callback(self, msg):
        if not self.allow_hardware_actuation:
            self.get_logger().warning(
                'Rejected gripper pulse: hardware actuation is disabled'
            )
            return

        pulse_us = msg.data

        min_pulse = GRIPPER_MIN_PULSE_US
        max_pulse = GRIPPER_MAX_PULSE_US

        if pulse_us < min_pulse or pulse_us > max_pulse:
            self.get_logger().warn(
                f'Gripper pulse out of range: {pulse_us}us'
                f'[{min_pulse}, {max_pulse}]'
            )

            return

        command = (
            f'SET '
            f'{self.gripper_channel} '
            f'{pulse_us} '
            f'{self.move_duration_ms}'
        )

        if not self.send_gripper_command(command):
            return

        expected = (
            f'OK SET '
            f'{self.gripper_channel} '
            f'{pulse_us} '
            f'{self.move_duration_ms}'
        )

        reply = self.read_serial_reply(
            expected,
            2.0
        )

        if expected not in reply:
            self.get_logger().error(
                f'Gripper command was not acknowledged: {reply!r}'
            )
            return

        self.get_logger().info(
            f'[RAW REPLY] {reply!r}'
        )

    def send_gripper_command(self, command):
        if not self.allow_hardware_actuation:
            self.get_logger().warning(
                'Rejected gripper serial command: hardware actuation is disabled'
            )
            return False

        if not command.endswith('\n'):
            command += '\n'

        try:
            self.serial.write(
                command.encode('ascii')
            )

            self.serial.flush()

            self.get_logger().info(
                f'[SERIAL SEND] {command.strip()}'
            )

            return True

        except serial.SerialException as error:
            self.get_logger().error(
                f'Failed to send gripper command: {error}'
            )

            return False

    def read_serial_reply(self, expected, timeout):

        deadline = time.monotonic() + timeout
        received = bytearray()

        while time.monotonic() < deadline:

            received.extend(
                self.serial.read(256)
            )

            text = received.decode(
                'utf-8',
                errors='replace'
            )

            if expected in text:
                return text

            if 'ERR ' in text:
                return text

        return received.decode(
            'utf-8',
            errors='replace'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
