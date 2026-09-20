import math
from bisect import bisect_left


GRIPPER_MIN_PULSE_US = 600
GRIPPER_MAX_PULSE_US = 1700


TCP_CALIBRATION = (
    (600, 100.0),
    (700, 104.0),
    (800, 109.0),
    (900, 113.0),
    (1000, 116.0),
    (1100, 120.0),
    (1200, 123.0),
    (1300, 126.0),
    (1400, 128.0),
    (1500, 130.0),
    (1600, 131.0),
    (1700, 131.5),
)


def pulse_to_tcp_extension_mm(pulse_us):
    """Convert gripper pulse width to tool0-to-TCP extension in millimeters."""

    pulse = float(pulse_us)

    if not math.isfinite(pulse):
        raise ValueError(
            'gripper pulse must be finite'
        )

    if (
        pulse < GRIPPER_MIN_PULSE_US
        or pulse > GRIPPER_MAX_PULSE_US
    ):
        raise ValueError(
            f'gripper pulse must be between '
            f'{GRIPPER_MIN_PULSE_US} and '
            f'{GRIPPER_MAX_PULSE_US} us'
        )

    pulses = [
        point[0]
        for point in TCP_CALIBRATION
    ]

    index = bisect_left(
        pulses,
        pulse
    )

    # Exactly at the final calibration point.
    if index == len(TCP_CALIBRATION):
        return TCP_CALIBRATION[-1][1]

    pulse_high, tcp_high = (
        TCP_CALIBRATION[index]
    )

    # Exactly at a measured calibration point.
    if pulse == pulse_high:
        return tcp_high

    pulse_low, tcp_low = (
        TCP_CALIBRATION[index - 1]
    )

    ratio = (
        (pulse - pulse_low)
        / (pulse_high - pulse_low)
    )

    return (
        tcp_low
        + ratio
        * (tcp_high - tcp_low)
    )


def pulse_to_tcp_extension_m(pulse_us):
    """Convert gripper pulse width to tool0-to-TCP extension in meters."""

    return (
        pulse_to_tcp_extension_mm(
            pulse_us
        )
        / 1000.0
    )

def pulse_to_tcp_joint_position_m(pulse_us):
    """Convert gripper pulse to tcp_extension_joint position."""

    tcp_extension_m = (
        pulse_to_tcp_extension_m(
            pulse_us
        )
    )

    return (
        tcp_extension_m
        - 0.100
    )