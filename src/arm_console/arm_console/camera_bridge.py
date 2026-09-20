"""
Thread-isolated RGB and depth subscriptions for Arm Console.

This module deliberately depends only on the standard ``sensor_msgs`` camera
interfaces.  The Astra process may be launched by the integration launch file
or by an external owner; the bridge never opens USB devices and never starts or
stops a camera driver.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Optional

import numpy as np
from python_qt_binding.QtCore import QObject, Signal
from python_qt_binding.QtGui import QImage
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


CAMERA_DRIVER_NODE_NAMES = frozenset({'astra_camera_node', 'camera'})
RGB_ENCODINGS = {
    'rgb8': 3,
    'bgr8': 3,
    'rgba8': 4,
    'bgra8': 4,
    'mono8': 1,
}
DEPTH_ENCODINGS = {
    '16uc1': 2,
    'mono16': 2,
    '32fc1': 4,
}


@dataclass(frozen=True)
class CameraFrame:
    """One detached image ready to cross from ROS to the Qt thread."""

    image: QImage
    width: int
    height: int
    encoding: str
    frame_id: str
    received_at: float


@dataclass(frozen=True)
class CameraStreamStatus:
    """Health and metadata for one image stream."""

    fresh: bool
    received: bool
    age_sec: Optional[float]
    fps: float
    width: int
    height: int
    encoding: str
    frame_id: str
    publishers: int


@dataclass(frozen=True)
class CameraStatus:
    """Combined camera state without implying ownership of the driver."""

    state: str
    driver_detected: bool
    rgb: CameraStreamStatus
    depth: CameraStreamStatus


class CameraSignals(QObject):
    """Signals emitted by the dedicated camera executor thread."""

    rgb_frame = Signal(object)
    depth_frame = Signal(object)
    status_changed = Signal(object)
    log_message = Signal(str, str)


class LatestFrameStore:
    """Keep at most one unconsumed frame per stream across threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._frames = {'rgb': None, 'depth': None}

    def put(self, stream, frame):
        """Atomically replace the older unconsumed frame for ``stream``."""
        if stream not in self._frames:
            raise ValueError(f'unknown camera stream: {stream}')
        with self._lock:
            self._frames[stream] = frame

    def take(self, stream):
        """Return and clear the newest frame for ``stream``."""
        if stream not in self._frames:
            raise ValueError(f'unknown camera stream: {stream}')
        with self._lock:
            frame = self._frames[stream]
            self._frames[stream] = None
        return frame


class FrameRateEstimator:
    """Estimate receive FPS over a short monotonic-time window."""

    def __init__(self, horizon_sec=2.0, max_samples=240):
        if horizon_sec <= 0.0:
            raise ValueError('FPS horizon must be positive')
        self.horizon_sec = float(horizon_sec)
        self._samples = deque(maxlen=max_samples)

    def record(self, received_at):
        received_at = float(received_at)
        self._samples.append(received_at)
        while (
            len(self._samples) > 1
            and received_at - self._samples[0] > self.horizon_sec
        ):
            self._samples.popleft()

    @property
    def fps(self):
        if len(self._samples) < 2:
            return 0.0
        elapsed = self._samples[-1] - self._samples[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self._samples) - 1) / elapsed


class StreamTracker:
    """Track valid-frame freshness independently for RGB and depth."""

    def __init__(self):
        self.last_received_at = None
        self.width = 0
        self.height = 0
        self.encoding = ''
        self.frame_id = ''
        self.rate = FrameRateEstimator()

    def record(self, message, received_at):
        self.last_received_at = float(received_at)
        self.width = int(message.width)
        self.height = int(message.height)
        self.encoding = str(message.encoding)
        self.frame_id = str(message.header.frame_id)
        self.rate.record(received_at)

    def snapshot(self, now, timeout_sec, publishers=0):
        age_sec = None
        if self.last_received_at is not None:
            age_sec = max(0.0, float(now) - self.last_received_at)
        return CameraStreamStatus(
            fresh=age_sec is not None and age_sec <= timeout_sec,
            received=self.last_received_at is not None,
            age_sec=age_sec,
            fps=self.rate.fps,
            width=self.width,
            height=self.height,
            encoding=self.encoding,
            frame_id=self.frame_id,
            publishers=max(0, int(publishers)),
        )


def classify_camera_state(driver_detected, rgb_status, depth_status):
    """Classify camera health using frame freshness as the authority."""
    if rgb_status.fresh and depth_status.fresh:
        return 'streaming'
    if rgb_status.fresh:
        return 'rgb_only'
    if depth_status.fresh:
        return 'depth_only'
    if rgb_status.received or depth_status.received:
        return 'stream_lost'
    if (
        driver_detected
        or rgb_status.publishers > 0
        or depth_status.publishers > 0
    ):
        return 'waiting'
    return 'driver_absent'


def _validate_raster(message, encodings):
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    if encoding not in encodings:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    bytes_per_pixel = encodings[encoding]
    if step < width * bytes_per_pixel:
        raise ValueError('image step is smaller than one packed row')
    if len(message.data) < step * height:
        raise ValueError('image data is shorter than step * height')
    return width, height, step, encoding


def _validate_depth_raster(message):
    width, height, step, encoding = _validate_raster(
        message,
        DEPTH_ENCODINGS,
    )
    if encoding in ('16uc1', 'mono16') and step % 2:
        raise ValueError('16-bit depth step must be even')
    if encoding == '32fc1' and step % 4:
        raise ValueError('32-bit depth step must be divisible by four')
    return width, height, step, encoding


def rgb_message_to_qimage(message):
    """Convert a standard RGB-like Image message into detached QImage data."""
    width, height, step, encoding = _validate_raster(
        message,
        RGB_ENCODINGS,
    )
    raw = bytes(message.data[:step * height])

    if encoding == 'mono8':
        image = QImage(raw, width, height, step, QImage.Format_Grayscale8)
    elif encoding in ('rgb8', 'bgr8'):
        image = QImage(raw, width, height, step, QImage.Format_RGB888)
        if encoding == 'bgr8':
            image = image.rgbSwapped()
    else:
        image = QImage(raw, width, height, step, QImage.Format_RGBA8888)
        if encoding == 'bgra8':
            image = image.rgbSwapped()

    return image.copy()


def _depth_array_metres(message):
    width, height, step, encoding = _validate_depth_raster(message)
    if encoding in ('16uc1', 'mono16'):
        dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
        row_elements = step // 2
        raw = bytes(message.data[:step * height])
        depth = np.frombuffer(raw, dtype=dtype).reshape(height, row_elements)
        return depth[:, :width].astype(np.float32) / 1000.0

    dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
    row_elements = step // 4
    raw = bytes(message.data[:step * height])
    depth = np.frombuffer(raw, dtype=dtype).reshape(height, row_elements)
    return depth[:, :width].astype(np.float32)


def depth_message_to_qimage(message, min_depth_m=0.2, max_depth_m=5.0):
    """Render 16UC1/32FC1 metric depth as a detached pseudo-colour QImage."""
    min_depth_m = float(min_depth_m)
    max_depth_m = float(max_depth_m)
    if not math.isfinite(min_depth_m) or not math.isfinite(max_depth_m):
        raise ValueError('depth display range must be finite')
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError('invalid depth display range')

    depth = _depth_array_metres(message)
    valid = np.isfinite(depth) & (depth > 0.0)
    normalised = np.zeros(depth.shape, dtype=np.float32)
    normalised[valid] = np.clip(
        (depth[valid] - min_depth_m) / (max_depth_m - min_depth_m),
        0.0,
        1.0,
    )

    # A small vectorised blue-cyan-yellow-red map. Invalid depth remains black.
    colour_position = 1.0 - normalised
    red = np.clip(1.5 - np.abs(4.0 * colour_position - 3.0), 0.0, 1.0)
    green = np.clip(1.5 - np.abs(4.0 * colour_position - 2.0), 0.0, 1.0)
    blue = np.clip(1.5 - np.abs(4.0 * colour_position - 1.0), 0.0, 1.0)
    rgb = np.stack((red, green, blue), axis=-1)
    rgb[~valid] = 0.0
    rgb = np.ascontiguousarray(np.round(rgb * 255.0).astype(np.uint8))

    height, width = depth.shape
    raw = rgb.tobytes()
    image = QImage(raw, width, height, width * 3, QImage.Format_RGB888)
    return image.copy()


class CameraNode(Node):
    """ROS node kept separate from the safety-critical arm executor."""

    def __init__(
        self,
        signals,
        node_name='arm_console_camera_bridge',
        camera_label='CAMERA',
        driver_node_fqn=None,
        rgb_topic='/camera/color/image_raw',
        depth_topic='/camera/depth/image_raw',
        stream_timeout_sec=1.0,
        status_period_sec=0.2,
        max_display_fps=15.0,
        min_depth_m=0.2,
        max_depth_m=5.0,
        frame_sink=None,
        context=None,
    ):
        node_name = str(node_name).strip()
        camera_label = str(camera_label).strip()
        if not camera_label:
            raise ValueError('camera label must not be empty')
        super().__init__(
            node_name,
            context=context,
            use_global_arguments=False,
        )
        stream_timeout_sec = float(stream_timeout_sec)
        status_period_sec = float(status_period_sec)
        max_display_fps = float(max_display_fps)
        min_depth_m = float(min_depth_m)
        max_depth_m = float(max_depth_m)
        if not math.isfinite(stream_timeout_sec) or stream_timeout_sec <= 0.0:
            raise ValueError('camera stream timeout must be positive')
        if not math.isfinite(status_period_sec) or status_period_sec <= 0.0:
            raise ValueError('camera status period must be positive')
        if not math.isfinite(max_display_fps) or max_display_fps <= 0.0:
            raise ValueError('camera display FPS must be positive')
        if (
            not math.isfinite(min_depth_m)
            or not math.isfinite(max_depth_m)
            or min_depth_m < 0.0
            or max_depth_m <= min_depth_m
        ):
            raise ValueError('invalid camera depth display range')

        self.signals = signals
        self._frame_sink = frame_sink
        self.camera_label = camera_label
        self.driver_node_fqn = self._normalise_node_fqn(driver_node_fqn)
        self.rgb_topic = str(rgb_topic)
        self.depth_topic = str(depth_topic)
        self.stream_timeout_sec = stream_timeout_sec
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self._emit_period_sec = 1.0 / max_display_fps
        self._last_frame_emitted_at = {'rgb': None, 'depth': None}
        self._last_warning_at = {'rgb': None, 'depth': None}
        self._trackers = {
            'rgb': StreamTracker(),
            'depth': StreamTracker(),
        }
        self._last_state = None

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self._rgb_callback,
            image_qos,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            image_qos,
        )
        self.status_timer = self.create_timer(
            status_period_sec,
            self._publish_status,
        )

    @staticmethod
    def _normalise_node_fqn(node_fqn):
        if node_fqn is None:
            return None
        node_fqn = str(node_fqn).strip()
        if not node_fqn:
            return None
        return '/' + node_fqn.strip('/')

    @staticmethod
    def _node_fqn(name, namespace):
        namespace = str(namespace).strip('/')
        name = str(name).strip('/')
        if namespace:
            return f'/{namespace}/{name}'
        return f'/{name}'

    def _should_emit_frame(self, stream, received_at):
        previous = self._last_frame_emitted_at[stream]
        if (
            previous is not None
            and received_at - previous < self._emit_period_sec
        ):
            return False
        self._last_frame_emitted_at[stream] = received_at
        return True

    def _warn_invalid_frame(self, stream, error, now):
        previous = self._last_warning_at[stream]
        if previous is not None and now - previous < 5.0:
            return
        self._last_warning_at[stream] = now
        self.signals.log_message.emit(
            'warning',
            f'{self.camera_label} · ignored invalid {stream.upper()} frame: '
            f'{error}',
        )

    def _rgb_callback(self, message):
        received_at = time.monotonic()
        try:
            _validate_raster(message, RGB_ENCODINGS)
            self._trackers['rgb'].record(message, received_at)
            if not self._should_emit_frame('rgb', received_at):
                return
            image = rgb_message_to_qimage(message)
        except (TypeError, ValueError) as error:
            self._warn_invalid_frame('rgb', error, received_at)
            return

        frame = CameraFrame(
            image=image,
            width=int(message.width),
            height=int(message.height),
            encoding=str(message.encoding),
            frame_id=str(message.header.frame_id),
            received_at=received_at,
        )
        if self._frame_sink is None:
            self.signals.rgb_frame.emit(frame)
        else:
            self._frame_sink('rgb', frame)

    def _depth_callback(self, message):
        received_at = time.monotonic()
        try:
            _validate_depth_raster(message)
            self._trackers['depth'].record(message, received_at)
            if not self._should_emit_frame('depth', received_at):
                return
            image = depth_message_to_qimage(
                message,
                self.min_depth_m,
                self.max_depth_m,
            )
        except (TypeError, ValueError) as error:
            self._warn_invalid_frame('depth', error, received_at)
            return

        frame = CameraFrame(
            image=image,
            width=int(message.width),
            height=int(message.height),
            encoding=str(message.encoding),
            frame_id=str(message.header.frame_id),
            received_at=received_at,
        )
        if self._frame_sink is None:
            self.signals.depth_frame.emit(frame)
        else:
            self._frame_sink('depth', frame)

    def _driver_detected(self):
        try:
            discovered = self.get_node_names_and_namespaces()
            if self.driver_node_fqn is not None:
                return any(
                    self._node_fqn(name, namespace)
                    == self.driver_node_fqn
                    for name, namespace in discovered
                )
            return any(
                name in CAMERA_DRIVER_NODE_NAMES
                and name != self.get_name()
                for name, _namespace in discovered
            )
        except Exception:
            return False

    def build_status(self, now=None):
        """Build one immutable status snapshot for the Qt side."""
        now = time.monotonic() if now is None else float(now)
        rgb_publishers = self.count_publishers(self.rgb_topic)
        depth_publishers = self.count_publishers(self.depth_topic)
        rgb_status = self._trackers['rgb'].snapshot(
            now,
            self.stream_timeout_sec,
            rgb_publishers,
        )
        depth_status = self._trackers['depth'].snapshot(
            now,
            self.stream_timeout_sec,
            depth_publishers,
        )
        driver_detected = (
            self._driver_detected()
            or rgb_publishers > 0
            or depth_publishers > 0
        )
        return CameraStatus(
            state=classify_camera_state(
                driver_detected,
                rgb_status,
                depth_status,
            ),
            driver_detected=driver_detected,
            rgb=rgb_status,
            depth=depth_status,
        )

    def _publish_status(self):
        status = self.build_status()
        self.signals.status_changed.emit(status)
        if status.state == self._last_state:
            return
        self._last_state = status.state
        level, message = {
            'streaming': ('info', 'RGB and depth streams are ready'),
            'rgb_only': ('warning', 'depth stream is unavailable'),
            'depth_only': ('warning', 'RGB stream is unavailable'),
            'stream_lost': ('warning', 'image streams were lost'),
            'waiting': ('info', 'driver detected; waiting for frames'),
            'driver_absent': ('info', 'driver is not detected'),
        }[status.state]
        self.signals.log_message.emit(
            level,
            f'{self.camera_label} · {message}',
        )


class CameraBridge:
    """Own a camera node/executor thread, but never own the camera process."""

    def __init__(self, context=None, **node_options):
        self.signals = CameraSignals()
        self._latest_frames = LatestFrameStore()
        camera_label = str(node_options.get('camera_label', 'CAMERA'))
        self.node = CameraNode(
            self.signals,
            frame_sink=self._latest_frames.put,
            context=context,
            **node_options,
        )
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self._spin,
            name=(
                f'{camera_label.lower().replace(" ", "-")}'
                '-executor'
            ),
            daemon=True,
        )
        self._lock = threading.Lock()
        self._started = False
        self._shutdown = False
        self._destroyed = False

    def take_latest_frame(self, stream):
        """Return the newest frame without queueing older GUI work."""
        return self._latest_frames.take(stream)

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as error:
            self.signals.log_message.emit(
                'error',
                f'{self.node.camera_label} · ROS executor stopped '
                f'unexpectedly: {error}',
            )

    def start(self):
        """Start the independent camera callback thread once."""
        with self._lock:
            if self._shutdown:
                raise RuntimeError('camera bridge is already shut down')
            if self._started:
                return
            self._started = True
            self.thread.start()

    def shutdown(self, timeout_sec=2.0):
        """Stop callbacks within a bounded wait; safe to call repeatedly."""
        timeout_sec = max(0.0, float(timeout_sec))
        deadline = time.monotonic() + timeout_sec
        with self._lock:
            first_shutdown = not self._shutdown
            self._shutdown = True
            started = self._started

        if first_shutdown:
            self.executor.shutdown(timeout_sec=timeout_sec)
        if started and self.thread.is_alive():
            remaining = max(0.0, deadline - time.monotonic())
            self.thread.join(timeout=remaining)
        if self.thread.is_alive():
            return False

        with self._lock:
            if self._destroyed:
                return True
            self._destroyed = True
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        self.node.destroy_node()
        return not self.thread.is_alive()
