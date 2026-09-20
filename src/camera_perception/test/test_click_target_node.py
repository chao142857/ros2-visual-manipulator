from sensor_msgs.msg import CameraInfo, Image

from camera_perception.click_target_node import validate_registered_streams


COLOR_FRAME = 'camera_color_optical_frame'


def make_image(width=640, height=480, frame=COLOR_FRAME):
    message = Image()
    message.width = width
    message.height = height
    message.header.frame_id = frame
    return message


def make_info(width=640, height=480, frame=COLOR_FRAME):
    message = CameraInfo()
    message.width = width
    message.height = height
    message.header.frame_id = frame
    return message


def validate(rgb=None, depth=None, info=None):
    return validate_registered_streams(
        rgb or make_image(),
        depth or make_image(),
        make_info() if info is None else info,
        COLOR_FRAME,
    )


def test_registered_streams_accept_matching_pixel_geometry():
    valid, reason = validate()
    assert valid
    assert reason == ''


def test_registered_streams_reject_missing_camera_info():
    valid, reason = validate_registered_streams(
        make_image(),
        make_image(),
        None,
        COLOR_FRAME,
    )
    assert not valid
    assert 'CameraInfo' in reason


def test_registered_streams_reject_size_mismatch():
    valid, reason = validate(depth=make_image(width=320))
    assert not valid
    assert 'size mismatch' in reason


def test_registered_streams_reject_frame_mismatch():
    valid, reason = validate(
        depth=make_image(frame='camera_depth_optical_frame')
    )
    assert not valid
    assert 'frame mismatch' in reason


def test_registered_streams_reject_unregistered_depth_frame():
    depth_frame = 'camera_depth_optical_frame'
    valid, reason = validate_registered_streams(
        make_image(frame=depth_frame),
        make_image(frame=depth_frame),
        make_info(frame=depth_frame),
        COLOR_FRAME,
    )
    assert not valid
    assert 'expected registered frame' in reason
