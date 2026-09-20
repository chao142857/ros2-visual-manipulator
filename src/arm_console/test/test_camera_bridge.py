import struct
import time

import pytest
from rclpy.context import Context
from sensor_msgs.msg import Image

from arm_console.camera_bridge import (
    CameraBridge,
    CameraStreamStatus,
    FrameRateEstimator,
    LatestFrameStore,
    StreamTracker,
    classify_camera_state,
    depth_message_to_qimage,
    rgb_message_to_qimage,
)
from arm_console.main import EYE_CAMERA_OPTIONS, WRIST_D405_OPTIONS


def make_image(width, height, encoding, step, data, bigendian=False):
    message = Image()
    message.width = width
    message.height = height
    message.encoding = encoding
    message.step = step
    message.is_bigendian = bigendian
    message.data = list(data)
    message.header.frame_id = 'camera_optical_frame'
    return message


def stream_status(*, fresh=False, received=False, publishers=0):
    return CameraStreamStatus(
        fresh=fresh,
        received=received,
        age_sec=0.1 if received else None,
        fps=30.0 if received else 0.0,
        width=640 if received else 0,
        height=480 if received else 0,
        encoding='rgb8' if received else '',
        frame_id='camera_optical_frame' if received else '',
        publishers=publishers,
    )


def test_main_uses_official_topics_and_distinct_depth_ranges():
    assert EYE_CAMERA_OPTIONS == {
        'node_name': 'arm_console_eye_camera_bridge',
        'camera_label': 'EYE CAMERA',
        'driver_node_fqn': '/camera/camera',
        'rgb_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/depth/image_raw',
        'min_depth_m': 0.2,
        'max_depth_m': 5.0,
    }
    assert WRIST_D405_OPTIONS == {
        'node_name': 'arm_console_wrist_d405_bridge',
        'camera_label': 'WRIST D405',
        'driver_node_fqn': '/wrist_camera/wrist_d405',
        'rgb_topic': '/wrist_camera/wrist_d405/color/image_rect_raw',
        'depth_topic': (
            '/wrist_camera/wrist_d405/depth/image_rect_raw'
        ),
        'min_depth_m': 0.05,
        'max_depth_m': 1.0,
    }
    assert EYE_CAMERA_OPTIONS['node_name'] != (
        WRIST_D405_OPTIONS['node_name']
    )


def test_rgb8_and_bgr8_are_detached_and_channel_correct():
    rgb_message = make_image(
        2,
        1,
        'rgb8',
        6,
        bytes((255, 0, 0, 0, 255, 0)),
    )
    rgb_image = rgb_message_to_qimage(rgb_message)

    assert rgb_image.pixelColor(0, 0).getRgb()[:3] == (255, 0, 0)
    assert rgb_image.pixelColor(1, 0).getRgb()[:3] == (0, 255, 0)

    bgr_message = make_image(
        1,
        1,
        'bgr8',
        3,
        bytes((255, 0, 0)),
    )
    bgr_image = rgb_message_to_qimage(bgr_message)
    bgr_message.data[0] = 0
    bgr_message.data[1] = 0
    bgr_message.data[2] = 0

    assert bgr_image.pixelColor(0, 0).getRgb()[:3] == (0, 0, 255)


def test_depth_16uc1_is_pseudocoloured_and_invalid_zero_is_black():
    message = make_image(
        3,
        1,
        '16UC1',
        6,
        struct.pack('<HHH', 0, 1000, 4000),
    )

    image = depth_message_to_qimage(message, 0.2, 5.0)

    assert image.pixelColor(0, 0).getRgb()[:3] == (0, 0, 0)
    assert image.pixelColor(1, 0).getRgb()[:3] != (0, 0, 0)
    assert (
        image.pixelColor(1, 0).getRgb()[:3]
        != image.pixelColor(2, 0).getRgb()[:3]
    )


def test_depth_32fc1_honours_big_endian_and_padding():
    row = struct.pack('>ff', 1.0, 2.0) + b'pad!'
    message = make_image(2, 1, '32FC1', 12, row, bigendian=True)

    image = depth_message_to_qimage(message)

    assert not image.isNull()
    assert image.width() == 2
    assert image.pixelColor(0, 0) != image.pixelColor(1, 0)


@pytest.mark.parametrize(
    'encoding,step,data,error_text',
    [
        ('16UC1', 3, b'\x00\x00\x00', 'even'),
        ('32FC1', 6, b'\x00' * 6, 'divisible by four'),
    ],
)
def test_invalid_depth_row_alignment_is_rejected_before_tracking(
    encoding,
    step,
    data,
    error_text,
):
    context = Context()
    context.init(initialize_logging=False)
    bridge = None
    try:
        bridge = CameraBridge(context=context)
        message = make_image(1, 1, encoding, step, data)

        with pytest.raises(ValueError, match=error_text):
            depth_message_to_qimage(message)
        bridge.node._depth_callback(message)

        assert bridge.node._trackers['depth'].last_received_at is None
    finally:
        if bridge is not None:
            bridge.shutdown(timeout_sec=0.2)
        if context.ok():
            context.shutdown()


@pytest.mark.parametrize(
    'message,error_text',
    [
        (make_image(1, 1, 'yuv422', 2, b'xx'), 'unsupported'),
        (make_image(2, 1, 'rgb8', 3, b'xxx'), 'step'),
        (make_image(2, 2, 'mono8', 2, b'xx'), 'shorter'),
    ],
)
def test_invalid_rgb_layout_is_rejected(message, error_text):
    with pytest.raises(ValueError, match=error_text):
        rgb_message_to_qimage(message)


def test_camera_state_uses_fresh_frames_not_endpoint_presence():
    missing = stream_status()
    stale = stream_status(received=True, publishers=1)
    rgb_fresh = stream_status(fresh=True, received=True, publishers=1)
    depth_fresh = stream_status(fresh=True, received=True, publishers=1)

    assert classify_camera_state(False, missing, missing) == 'driver_absent'
    assert classify_camera_state(True, missing, missing) == 'waiting'
    assert classify_camera_state(True, rgb_fresh, missing) == 'rgb_only'
    assert classify_camera_state(True, missing, depth_fresh) == 'depth_only'
    assert classify_camera_state(True, rgb_fresh, depth_fresh) == 'streaming'
    assert classify_camera_state(True, stale, stale) == 'stream_lost'


def test_stream_tracker_reports_receive_fps_and_monotonic_age():
    tracker = StreamTracker()
    message = make_image(1, 1, 'mono8', 1, b'x')
    tracker.record(message, 10.0)
    tracker.record(message, 10.1)
    tracker.record(message, 10.2)

    status = tracker.snapshot(10.25, timeout_sec=0.5, publishers=1)

    assert status.fresh
    assert status.age_sec == pytest.approx(0.05)
    assert status.fps == pytest.approx(10.0)
    assert status.width == 1
    assert status.frame_id == 'camera_optical_frame'


def test_frame_rate_estimator_handles_duplicate_timestamps():
    estimator = FrameRateEstimator()
    estimator.record(1.0)
    estimator.record(1.0)

    assert estimator.fps == 0.0


def test_latest_frame_store_drops_superseded_frames():
    store = LatestFrameStore()

    store.put('rgb', 'old')
    store.put('rgb', 'latest')
    store.put('depth', 'depth')

    assert store.take('rgb') == 'latest'
    assert store.take('rgb') is None
    assert store.take('depth') == 'depth'
    with pytest.raises(ValueError, match='unknown camera stream'):
        store.take('infrared')


def test_camera_bridge_starts_and_shuts_down_without_hardware():
    context = Context()
    context.init(initialize_logging=False)
    bridge = None
    try:
        bridge = CameraBridge(
            context=context,
            status_period_sec=0.05,
            max_display_fps=5.0,
        )
        bridge.start()
        bridge.start()
        started_at = time.monotonic()

        assert bridge.shutdown(timeout_sec=1.0)
        assert time.monotonic() - started_at < 1.5
        assert not bridge.thread.is_alive()
        assert bridge.shutdown(timeout_sec=0.0)
    finally:
        if bridge is not None:
            bridge.shutdown(timeout_sec=0.2)
        if context.ok():
            context.shutdown()


def test_camera_node_ignores_console_process_name_remap():
    context = Context()
    context.init(
        args=['--ros-args', '-r', '__node:=arm_console'],
        initialize_logging=False,
    )
    bridge = None
    try:
        bridge = CameraBridge(context=context)
        assert bridge.node.get_fully_qualified_name() == (
            '/arm_console_camera_bridge'
        )
    finally:
        if bridge is not None:
            bridge.shutdown(timeout_sec=0.2)
        if context.ok():
            context.shutdown()


def test_camera_node_accepts_unique_name_topics_and_exact_driver_fqn(
    monkeypatch,
):
    context = Context()
    context.init(initialize_logging=False)
    bridge = None
    try:
        bridge = CameraBridge(
            context=context,
            node_name='arm_console_wrist_d405_bridge',
            camera_label='WRIST D405',
            driver_node_fqn='wrist_camera/wrist_d405/',
            rgb_topic='/wrist_camera/wrist_d405/color/image_rect_raw',
            depth_topic=(
                '/wrist_camera/wrist_d405/depth/image_rect_raw'
            ),
            min_depth_m=0.05,
            max_depth_m=1.0,
        )
        monkeypatch.setattr(
            bridge.node,
            'get_node_names_and_namespaces',
            lambda: [
                ('camera', '/camera'),
                ('wrist_d405', '/wrist_camera'),
            ],
        )

        assert bridge.node.get_fully_qualified_name() == (
            '/arm_console_wrist_d405_bridge'
        )
        assert bridge.node.camera_label == 'WRIST D405'
        assert bridge.node.driver_node_fqn == '/wrist_camera/wrist_d405'
        assert bridge.node.rgb_topic == (
            '/wrist_camera/wrist_d405/color/image_rect_raw'
        )
        assert bridge.node.depth_topic == (
            '/wrist_camera/wrist_d405/depth/image_rect_raw'
        )
        assert bridge.node.min_depth_m == pytest.approx(0.05)
        assert bridge.node.max_depth_m == pytest.approx(1.0)
        assert bridge.node._driver_detected()

        monkeypatch.setattr(
            bridge.node,
            'get_node_names_and_namespaces',
            lambda: [('camera', '/camera')],
        )
        assert not bridge.node._driver_detected()
    finally:
        if bridge is not None:
            bridge.shutdown(timeout_sec=0.2)
        if context.ok():
            context.shutdown()


def test_camera_transition_and_invalid_frame_logs_include_role():
    context = Context()
    context.init(initialize_logging=False)
    bridge = None
    logs = []
    try:
        bridge = CameraBridge(
            context=context,
            node_name='arm_console_wrist_d405_bridge',
            camera_label='WRIST D405',
            driver_node_fqn='/wrist_camera/wrist_d405',
        )
        bridge.signals.log_message.connect(
            lambda level, message: logs.append((level, message))
        )

        bridge.node._publish_status()
        bridge.node._rgb_callback(
            make_image(1, 1, 'yuv422', 2, b'xx')
        )

        assert logs
        assert all(message.startswith('WRIST D405 ·') for _, message in logs)
        assert any('driver is not detected' in message for _, message in logs)
        assert any(
            'ignored invalid RGB frame' in message
            for _, message in logs
        )
    finally:
        if bridge is not None:
            bridge.shutdown(timeout_sec=0.2)
        if context.ok():
            context.shutdown()


def test_two_camera_bridges_have_independent_executors_and_shutdown():
    context = Context()
    context.init(initialize_logging=False)
    eye = None
    wrist = None
    try:
        eye = CameraBridge(
            context=context,
            node_name='arm_console_eye_camera_bridge',
            camera_label='EYE CAMERA',
            driver_node_fqn='/camera/camera',
        )
        wrist = CameraBridge(
            context=context,
            node_name='arm_console_wrist_d405_bridge',
            camera_label='WRIST D405',
            driver_node_fqn='/wrist_camera/wrist_d405',
            rgb_topic='/wrist_camera/wrist_d405/color/image_rect_raw',
            depth_topic=(
                '/wrist_camera/wrist_d405/depth/image_rect_raw'
            ),
            min_depth_m=0.05,
            max_depth_m=1.0,
        )

        assert eye.executor is not wrist.executor
        assert eye.thread is not wrist.thread
        assert eye.thread.name == 'eye-camera-executor'
        assert wrist.thread.name == 'wrist-d405-executor'
        assert eye.node.get_fully_qualified_name() != (
            wrist.node.get_fully_qualified_name()
        )

        eye.start()
        wrist.start()
        assert eye.thread.is_alive()
        assert wrist.thread.is_alive()

        assert wrist.shutdown(timeout_sec=1.0)
        assert not wrist.thread.is_alive()
        assert eye.thread.is_alive()
        assert eye.shutdown(timeout_sec=1.0)
        assert not eye.thread.is_alive()
    finally:
        if wrist is not None:
            wrist.shutdown(timeout_sec=0.2)
        if eye is not None:
            eye.shutdown(timeout_sec=0.2)
        if context.ok():
            context.shutdown()
