import os
import pty
import threading
import time

import pytest
import serial

from arm_driver.arm_driver_node import ArmResponseTimeout
from test_start_handshake import make_node


def open_pty_serial():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    os.close(slave_fd)
    port = serial.Serial(
        port=slave_name,
        baudrate=115200,
        timeout=0.01,
        write_timeout=0.05,
        exclusive=True,
    )
    return master_fd, port


def test_real_pty_handles_fragmented_and_interleaved_responses():
    master_fd, port = open_pty_serial()
    node = make_node()
    node.serial_port = port
    node.start_ack_timeout_sec = 0.2
    node.command_ack_timeout_sec = 0.2
    node.joint_response_timeout_sec = 0.2
    received_commands = []

    def emulate_controller():
        receive_buffer = bytearray()
        while len(received_commands) < 3:
            receive_buffer.extend(os.read(master_fd, 256))
            while b'\n' in receive_buffer:
                command, _, remainder = receive_buffer.partition(b'\n')
                receive_buffer[:] = remainder
                received_commands.append(command + b'\n')

                if command == b'!START':
                    os.write(master_fd, b'Started ')
                    time.sleep(0.002)
                    os.write(master_fd, b'ok\r\n')
                elif command.startswith(b'&'):
                    os.write(master_fd, b'15')
                    os.write(master_fd, b'ok\r\n\r\n')
                elif command == b'#GETJPOS':
                    os.write(master_fd, b'ok 1 2 3 ')
                    time.sleep(0.002)
                    os.write(master_fd, b'4 5 6ok\r\n\r\n')

    emulator = threading.Thread(target=emulate_controller, daemon=True)
    emulator.start()

    try:
        assert node.send_joint_command('&1,2,3,4,5,0\n')
        with node.serial_lock:
            joints = node._query_joint_positions_locked()

        assert joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert received_commands == [
            b'!START\n',
            b'&1,2,3,4,5,0\n',
            b'#GETJPOS\n',
        ]
    finally:
        port.close()
        os.close(master_fd)
        emulator.join(timeout=0.2)


def test_real_pty_silent_read_has_total_deadline():
    master_fd, port = open_pty_serial()
    node = make_node()
    node.serial_port = port
    node.joint_response_timeout_sec = 0.04

    try:
        started_at = time.monotonic()
        with node.serial_lock:
            with pytest.raises(ArmResponseTimeout):
                node._query_joint_positions_locked()
        elapsed = time.monotonic() - started_at

        assert elapsed < 0.2
    finally:
        port.close()
        os.close(master_fd)
