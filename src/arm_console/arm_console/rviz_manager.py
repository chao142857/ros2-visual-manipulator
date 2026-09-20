"""Start and stop only the RViz processes owned by Arm Console."""

from collections import deque

from python_qt_binding.QtCore import QObject, QProcess, Signal


class RvizManager(QObject):
    """Manage RViz while reusing the existing arm_description assets."""

    status_changed = Signal(str)
    log_message = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.started.connect(self._process_started)
        self.process.finished.connect(self._process_finished)
        self.process.errorOccurred.connect(self._process_error)
        self.process.readyReadStandardOutput.connect(self._drain_output)
        self._output_tail = deque(maxlen=20)
        self._stopping = False

    @property
    def is_running(self):
        """Return true only for a visualization process owned by this GUI."""
        return self.process.state() != QProcess.NotRunning

    def start(self, robot_state_publisher_ready, rviz_detected):
        """Open the measured plus isolated target-preview visualization."""
        if self.is_running:
            self.log_message.emit('warning', 'RViz is already starting or running')
            return
        if rviz_detected:
            self.status_changed.emit('external')
            self.log_message.emit(
                'info',
                'External RViz detected; no duplicate process started',
            )
            return

        self._output_tail.clear()
        self._stopping = False
        self.status_changed.emit('starting')

        program = 'ros2'
        arguments = [
            'launch',
            'arm_console',
            'preview_rviz.launch.py',
            'start_physical_state_publisher:='
            + ('false' if robot_state_publisher_ready else 'true'),
        ]

        self.process.start(program, arguments)

    def stop(self):
        """Stop the visualization process owned by this GUI."""
        if not self.is_running:
            return
        self._stopping = True
        self.status_changed.emit('stopping')
        self.process.terminate()
        if not self.process.waitForFinished(2000):
            self.process.kill()
            self.process.waitForFinished(1000)

    def shutdown(self):
        """Clean up owned child processes during window shutdown."""
        self.stop()

    def _process_started(self):
        self.status_changed.emit('running')
        self.log_message.emit(
            'info',
            'RViz visualization and target preview started',
        )

    def _process_finished(self, exit_code, _exit_status):
        self.status_changed.emit('stopped')
        if self._stopping:
            self.log_message.emit('info', 'RViz visualization stopped')
            return

        if exit_code == 0:
            self.log_message.emit('info', 'RViz visualization closed')
            return

        detail = self._output_tail[-1] if self._output_tail else 'no output'
        self.log_message.emit(
            'error',
            f'RViz exited with code {exit_code}: {detail}',
        )

    def _process_error(self, error):
        self.status_changed.emit('stopped')
        self.log_message.emit(
            'error',
            f'Failed to start RViz: {self.process.errorString()} ({error})',
        )

    def _drain_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode(
            'utf-8',
            errors='replace',
        )
        for line in data.splitlines():
            stripped = line.strip()
            if stripped:
                self._output_tail.append(stripped)
