"""Reusable RGB/depth display panel for the subscriber-only camera bridge."""

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


CAMERA_STATE_LABELS = {
    'driver_absent': 'DRIVER ABSENT',
    'waiting': 'WAITING FOR CAMERA',
    'rgb_only': 'RGB ONLY',
    'depth_only': 'DEPTH ONLY',
    'streaming': 'STREAMING',
    'stream_lost': 'STREAM LOST',
}


def format_stream_metadata(status):
    """Return compact, stable metadata text for one stream status."""
    if not status.received:
        if status.publishers > 0:
            return 'Publisher detected · waiting for a valid frame'
        return 'No publisher · no valid frame received'

    age_text = (
        'unknown age'
        if status.age_sec is None
        else f'{status.age_sec:.2f} s ago'
    )
    dimensions = f'{status.width}×{status.height}'
    encoding = status.encoding.upper() or 'UNKNOWN'
    return f'{dimensions} · {encoding} · {status.fps:.1f} FPS · {age_text}'


class ImagePreview(QLabel):
    """Aspect-preserving image label with an explicit missing-stream state."""

    def __init__(self, placeholder, parent=None):
        super().__init__(parent)
        self._image = None
        self._placeholder = str(placeholder)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(180, 135)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(
            'background-color: #0f1419; border: 1px solid #303944; '
            'border-radius: 4px; color: #778696;'
        )
        self.setText(self._placeholder)

    @property
    def has_image(self):
        return self._image is not None

    def set_image(self, image):
        self._image = image
        self.setText('')
        self._render_image()

    def show_placeholder(self, text=None):
        self._image = None
        self.clear()
        self.setText(str(text or self._placeholder))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_image()

    def _render_image(self):
        if self._image is None or self._image.isNull():
            return
        pixmap = QPixmap.fromImage(self._image)
        self.setPixmap(
            pixmap.scaled(
                self.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )


class StreamView(QWidget):
    """One titled preview plus its live metadata."""

    def __init__(self, title, placeholder, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        heading = QLabel(title)
        heading.setObjectName('columnHeading')
        self.preview = ImagePreview(placeholder)
        self.metadata = QLabel('No valid frame received')
        self.metadata.setObjectName('secondaryText')
        self.metadata.setWordWrap(True)

        layout.addWidget(heading)
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.metadata)


class CameraPanel(QFrame):
    """Display camera streams without taking ownership of their process."""

    def __init__(
        self,
        bridge=None,
        render_fps=15.0,
        parent=None,
        title='Depth Camera',
        ownership_note=(
            'Subscriber only · camera lifecycle is managed outside this GUI.'
        ),
    ):
        super().__init__(parent)
        if render_fps <= 0.0:
            raise ValueError('camera panel render FPS must be positive')
        self.setProperty('card', True)
        self.setObjectName('cameraPanel')
        self._bridge = None
        self._frame_source = None
        self._status = None
        self._pending_rgb = None
        self._pending_depth = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)

        heading_row = QHBoxLayout()
        self.title_label = QLabel(str(title))
        self.title_label.setObjectName('sectionTitle')
        self.state_label = QLabel('DRIVER ABSENT')
        self.state_label.setObjectName('statusValue')
        heading_row.addWidget(self.title_label)
        heading_row.addStretch(1)
        heading_row.addWidget(self.state_label)
        layout.addLayout(heading_row)

        self.ownership_note = QLabel(str(ownership_note))
        self.ownership_note.setObjectName('secondaryText')
        self.ownership_note.setWordWrap(True)
        layout.addWidget(self.ownership_note)

        preview_grid = QGridLayout()
        preview_grid.setHorizontalSpacing(10)
        preview_grid.setColumnStretch(0, 1)
        preview_grid.setColumnStretch(1, 1)
        self.rgb_view = StreamView('RGB', 'WAITING FOR RGB')
        self.depth_view = StreamView('DEPTH', 'WAITING FOR DEPTH')
        preview_grid.addWidget(self.rgb_view, 0, 0)
        preview_grid.addWidget(self.depth_view, 0, 1)
        layout.addLayout(preview_grid, 1)

        self.render_timer = QTimer(self)
        self.render_timer.setInterval(max(1, round(1000.0 / render_fps)))
        self.render_timer.timeout.connect(self._render_pending_frames)
        self.render_timer.start()

        if bridge is not None:
            self.attach_bridge(bridge)

    def attach_bridge(self, bridge):
        """Attach once to an object exposing CameraSignals as ``signals``."""
        if self._bridge is bridge:
            return
        if self._bridge is not None:
            raise RuntimeError('camera panel is already attached to a bridge')
        self._bridge = bridge
        if hasattr(bridge, 'take_latest_frame'):
            self._frame_source = bridge
        else:
            bridge.signals.rgb_frame.connect(self._queue_rgb_frame)
            bridge.signals.depth_frame.connect(self._queue_depth_frame)
        bridge.signals.status_changed.connect(self._status_changed)

    def _queue_rgb_frame(self, frame):
        self._pending_rgb = frame

    def _queue_depth_frame(self, frame):
        self._pending_depth = frame

    def _render_pending_frames(self):
        if self._frame_source is not None:
            self._pending_rgb = self._frame_source.take_latest_frame('rgb')
            self._pending_depth = self._frame_source.take_latest_frame('depth')
        if self._pending_rgb is not None:
            frame = self._pending_rgb
            self._pending_rgb = None
            if self._status is None or self._status.rgb.fresh:
                self.rgb_view.preview.set_image(frame.image)
        if self._pending_depth is not None:
            frame = self._pending_depth
            self._pending_depth = None
            if self._status is None or self._status.depth.fresh:
                self.depth_view.preview.set_image(frame.image)

    def _status_changed(self, status):
        self._status = status
        self.state_label.setText(
            CAMERA_STATE_LABELS.get(status.state, status.state.upper())
        )
        attention = status.state in {
            'rgb_only',
            'depth_only',
            'stream_lost',
        }
        self.state_label.setProperty('attention', attention)
        self.state_label.style().unpolish(self.state_label)
        self.state_label.style().polish(self.state_label)

        self.rgb_view.metadata.setText(format_stream_metadata(status.rgb))
        self.depth_view.metadata.setText(format_stream_metadata(status.depth))
        self._update_missing_stream(
            status.rgb,
            self.rgb_view.preview,
            'RGB',
        )
        self._update_missing_stream(
            status.depth,
            self.depth_view.preview,
            'DEPTH',
        )

    def _update_missing_stream(self, stream_status, preview, name):
        if stream_status.fresh:
            return
        if name == 'RGB':
            self._pending_rgb = None
        else:
            self._pending_depth = None
        if stream_status.received:
            age = stream_status.age_sec
            age_text = '' if age is None else f' · {age:.1f} s'
            preview.show_placeholder(f'{name} STREAM LOST{age_text}')
        elif stream_status.publishers > 0:
            preview.show_placeholder(f'WAITING FOR VALID {name} FRAME')
        else:
            preview.show_placeholder(f'WAITING FOR {name}')

    def closeEvent(self, event):
        """Stop only this widget's timer; bridge ownership stays outside."""
        self.render_timer.stop()
        super().closeEvent(event)
