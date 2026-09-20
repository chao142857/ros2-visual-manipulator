from python_qt_binding.QtCore import QObject, Signal
from python_qt_binding.QtGui import QImage
from python_qt_binding.QtWidgets import QApplication, QPushButton

from arm_console.camera_bridge import (
    CameraFrame,
    CameraStatus,
    CameraStreamStatus,
)
from arm_console.camera_panel import CameraPanel, format_stream_metadata


class FakeCameraSignals(QObject):
    rgb_frame = Signal(object)
    depth_frame = Signal(object)
    status_changed = Signal(object)
    log_message = Signal(str, str)


class FakeCameraBridge:
    def __init__(self):
        self.signals = FakeCameraSignals()


class PullCameraBridge(FakeCameraBridge):
    def __init__(self):
        super().__init__()
        self.frames = {'rgb': None, 'depth': None}

    def take_latest_frame(self, stream):
        frame = self.frames[stream]
        self.frames[stream] = None
        return frame


def make_stream_status(
    *,
    fresh,
    received=True,
    age_sec=0.05,
    encoding='rgb8',
    publishers=1,
):
    return CameraStreamStatus(
        fresh=fresh,
        received=received,
        age_sec=age_sec if received else None,
        fps=29.8 if received else 0.0,
        width=640 if received else 0,
        height=480 if received else 0,
        encoding=encoding if received else '',
        frame_id='camera_optical_frame' if received else '',
        publishers=publishers,
    )


def make_frame(encoding):
    image = QImage(8, 6, QImage.Format_RGB888)
    image.fill(0xff336699)
    return CameraFrame(
        image=image,
        width=8,
        height=6,
        encoding=encoding,
        frame_id='camera_optical_frame',
        received_at=1.0,
    )


def test_stream_metadata_is_compact_and_reports_waiting_publishers():
    waiting = make_stream_status(
        fresh=False,
        received=False,
        publishers=1,
    )
    fresh = make_stream_status(fresh=True)

    assert 'waiting for a valid frame' in format_stream_metadata(waiting)
    assert '640×480' in format_stream_metadata(fresh)
    assert '29.8 FPS' in format_stream_metadata(fresh)


def test_camera_panel_renders_latest_frames_then_clears_on_stream_loss():
    application = QApplication.instance() or QApplication([])
    bridge = FakeCameraBridge()
    panel = CameraPanel(bridge, render_fps=15.0)
    rgb = make_stream_status(fresh=True, encoding='rgb8')
    depth = make_stream_status(fresh=True, encoding='16UC1')

    bridge.signals.status_changed.emit(
        CameraStatus(
            state='streaming',
            driver_detected=True,
            rgb=rgb,
            depth=depth,
        )
    )
    bridge.signals.rgb_frame.emit(make_frame('rgb8'))
    bridge.signals.depth_frame.emit(make_frame('16UC1'))
    application.processEvents()
    panel._render_pending_frames()

    assert panel.state_label.text() == 'STREAMING'
    assert panel.rgb_view.preview.has_image
    assert panel.depth_view.preview.has_image
    assert '16UC1' in panel.depth_view.metadata.text()

    stale_rgb = make_stream_status(
        fresh=False,
        age_sec=1.2,
        encoding='rgb8',
    )
    stale_depth = make_stream_status(
        fresh=False,
        age_sec=1.2,
        encoding='16UC1',
    )
    bridge.signals.status_changed.emit(
        CameraStatus(
            state='stream_lost',
            driver_detected=True,
            rgb=stale_rgb,
            depth=stale_depth,
        )
    )
    application.processEvents()

    assert panel.state_label.text() == 'STREAM LOST'
    assert not panel.rgb_view.preview.has_image
    assert not panel.depth_view.preview.has_image
    assert 'RGB STREAM LOST' in panel.rgb_view.preview.text()
    assert 'DEPTH STREAM LOST' in panel.depth_view.preview.text()
    panel.close()


def test_camera_panel_has_no_driver_start_or_stop_controls():
    application = QApplication.instance() or QApplication([])
    panel = CameraPanel(FakeCameraBridge())
    application.processEvents()

    assert panel.findChildren(QPushButton) == []
    panel.close()


def test_camera_panel_uses_configured_title_and_ownership_note():
    application = QApplication.instance() or QApplication([])
    panel = CameraPanel(
        FakeCameraBridge(),
        title='Wrist Camera · Intel RealSense D405',
        ownership_note=(
            'Subscriber only · RealSense lifecycle is externally owned.'
        ),
    )
    application.processEvents()

    assert panel.title_label.text() == (
        'Wrist Camera · Intel RealSense D405'
    )
    assert panel.ownership_note.text() == (
        'Subscriber only · RealSense lifecycle is externally owned.'
    )
    panel.close()


def test_camera_panel_pulls_frames_without_qt_frame_signal_queueing():
    application = QApplication.instance() or QApplication([])
    bridge = PullCameraBridge()
    panel = CameraPanel(bridge)
    fresh = make_stream_status(fresh=True)
    bridge.signals.status_changed.emit(
        CameraStatus(
            state='streaming',
            driver_detected=True,
            rgb=fresh,
            depth=fresh,
        )
    )
    bridge.frames['rgb'] = make_frame('rgb8')
    bridge.frames['depth'] = make_frame('16UC1')
    application.processEvents()

    panel._render_pending_frames()

    assert panel.rgb_view.preview.has_image
    assert panel.depth_view.preview.has_image
    assert bridge.frames == {'rgb': None, 'depth': None}
    panel.close()
