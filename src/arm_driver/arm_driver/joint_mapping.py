import math

RAW_REFERENCE_DEG = [
    0.0,
    -75.0,
    180.0,
    0.0,
    0.0,
    0.0,
]

IDEAL_REFERENCE_DEG = [
    0.0,
    75.0,
    -90.0,
    0.0,
    0.0,
    0.0,
]

JOINT_DIRECTION = [
    -1.0,
    -1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
]

# ideal joint degree / raw controller degree
JOINT_SCALE = [
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
]

ZERO_OFFSET_DEG = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]

def raw_deg_to_ideal_deg(raw_degrees):

    if len(raw_degrees) != 6:
        raise ValueError(
            f'Expected 6 joint angles,got {len(raw_degrees)}'
        )

    ideal_degrees = []

    for raw, raw_ref, ideal_ref, direction, scale, zero_offset in zip(
        raw_degrees,
        RAW_REFERENCE_DEG,
        IDEAL_REFERENCE_DEG,
        JOINT_DIRECTION,
        JOINT_SCALE,
        ZERO_OFFSET_DEG,
    ):
        old_ideal = ideal_ref + direction * (raw - raw_ref) * scale
        ideal = old_ideal - zero_offset
        ideal_degrees.append(ideal)

    return ideal_degrees


def raw_deg_to_ideal_rad(raw_degrees):
    ideal_degrees = raw_deg_to_ideal_deg(
        raw_degrees
    )

    return [
        math.radians(angle)
        for angle in ideal_degrees
    ]

def ideal_deg_to_raw_deg(ideal_degrees):
    if len(ideal_degrees) != 6:
        raise ValueError(
            f'Expected 6 joint angles,got {len(ideal_degrees)}'
        )

    raw_degrees = []

    for ideal, raw_ref, ideal_ref, direction, scale, zero_offset in zip(
        ideal_degrees,
        RAW_REFERENCE_DEG,
        IDEAL_REFERENCE_DEG,
        JOINT_DIRECTION,
        JOINT_SCALE,
        ZERO_OFFSET_DEG,
    ):
        old_ideal = ideal + zero_offset
        raw = raw_ref + direction * (old_ideal - ideal_ref) / scale
        raw_degrees.append(raw)

    return raw_degrees

def ideal_rad_to_raw_deg(ideal_radians):
    ideal_degrees = [
        math.degrees(angle)
        for angle in ideal_radians
    ]

    return ideal_deg_to_raw_deg(ideal_degrees)