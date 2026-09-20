import math
import struct

from sensor_msgs.msg import CameraInfo, Image

from dual_camera_foundation.rgbd_target_point import (
    depth_at,
    intrinsics,
    median_depth,
    nearest_depth_frame,
    stamp_nanoseconds,
)


def _depth_image(encoding, values, width, height, big_endian=False):
    message = Image()
    message.width = width
    message.height = height
    message.encoding = encoding
    message.is_bigendian = big_endian
    item_size = 2 if encoding == '16UC1' else 4
    message.step = width * item_size
    code = 'H' if encoding == '16UC1' else 'f'
    order = '>' if big_endian else '<'
    message.data = struct.pack(order + code * len(values), *values)
    return message


def _stamped_depth(sec, nanosec, value=1000):
    message = _depth_image('16UC1', [value], 1, 1)
    message.header.frame_id = 'wrist_d405_color_optical_frame'
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    return message


def test_depth_at_handles_realsense_z16_and_float_depth():
    z16 = _depth_image('16UC1', [0, 1250, 2000, 3000], 2, 2)
    assert depth_at(z16, 1, 0) == 1.25
    assert depth_at(z16, 0, 0) is None
    assert depth_at(z16, 2, 0) is None

    float_depth = _depth_image(
        '32FC1', [0.4, float('nan'), -1.0, 1.75], 2, 2, True
    )
    assert math.isclose(depth_at(float_depth, 1, 1), 1.75)
    assert depth_at(float_depth, 1, 0) is None


def test_median_depth_ignores_invalid_pixels():
    message = _depth_image(
        '16UC1',
        [0, 1000, 0, 1200, 900, 1100, 0, 1300, 0],
        3,
        3,
    )
    assert median_depth(message, 1, 1, radius=1) == 1.1


def test_intrinsics_fall_back_from_k_to_p():
    message = CameraInfo()
    message.p = [
        500.0, 0.0, 320.0, 0.0,
        0.0, 510.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    assert intrinsics(message) == (500.0, 510.0, 320.0, 240.0)


def test_nearest_depth_frame_selects_closest_and_prefers_newer_tie():
    frames = [
        _stamped_depth(10, 0),
        _stamped_depth(10, 20_000_000),
        _stamped_depth(10, 40_000_000),
    ]
    pixel_stamp = frames[0].header.stamp.__class__()
    pixel_stamp.sec = 10
    pixel_stamp.nanosec = 27_000_000
    selected = nearest_depth_frame(frames, pixel_stamp, 0.05, 0.05)
    assert selected.header.stamp.nanosec == 20_000_000

    pixel_stamp.nanosec = 30_000_000
    selected = nearest_depth_frame(frames, pixel_stamp, 0.05, 0.05)
    assert selected.header.stamp.nanosec == 40_000_000


def test_nearest_depth_frame_rejects_frames_outside_tight_window():
    frames = [
        _stamped_depth(9, 900_000_000),
        _stamped_depth(10, 80_000_000),
    ]
    pixel_stamp = frames[0].header.stamp.__class__()
    pixel_stamp.sec = 10
    pixel_stamp.nanosec = 0
    assert nearest_depth_frame(frames, pixel_stamp, 0.05, 0.05) is None

    invalid = frames[0].header.stamp.__class__()
    assert stamp_nanoseconds(invalid) is None
    assert nearest_depth_frame(frames, invalid, 0.05, 0.05) is None


def test_rgbd_source_uses_depth_timestamp_for_tf():
    from pathlib import Path

    source = (
        Path(__file__).parents[1]
        / 'dual_camera_foundation'
        / 'rgbd_target_point.py'
    ).read_text()
    assert 'output.header = depth_message.header' in source
    assert 'output.header.stamp = message.header.stamp' not in source
