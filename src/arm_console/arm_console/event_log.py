"""Best-effort persistent event logging for the desktop console."""

from logging import Formatter, LogRecord
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import sys
from typing import Optional


LOG_FILE_NAME = 'arm_console_events.log'
DEFAULT_MAX_BYTES = 1024 * 1024
DEFAULT_BACKUP_COUNT = 3


def ros_log_directory() -> Path:
    """Return the directory used by ROS 2 for process logs."""
    configured_log_dir = os.environ.get('ROS_LOG_DIR')
    if configured_log_dir:
        return Path(configured_log_dir).expanduser()

    configured_ros_home = os.environ.get('ROS_HOME')
    ros_home = (
        Path(configured_ros_home).expanduser()
        if configured_ros_home
        else Path.home() / '.ros'
    )
    return ros_home / 'log'


class _NonFatalRotatingFileHandler(RotatingFileHandler):
    """Capture filesystem errors instead of surfacing them through Qt."""

    def __init__(self, *args, **kwargs):
        self.last_error: Optional[BaseException] = None
        super().__init__(*args, **kwargs)

    def handleError(self, _record):  # noqa: N802 - logging API spelling.
        self.last_error = sys.exc_info()[1]


class EventLogWriter:
    """Write a bounded event history; all filesystem failures are non-fatal."""

    def __init__(
        self,
        directory=None,
        *,
        max_bytes=DEFAULT_MAX_BYTES,
        backup_count=DEFAULT_BACKUP_COUNT,
    ):
        self.path = Path(directory or ros_log_directory()) / LOG_FILE_NAME
        self.error = None
        self._handler = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            handler = _NonFatalRotatingFileHandler(
                self.path,
                maxBytes=max_bytes,
                backupCount=backup_count,
                encoding='utf-8',
            )
            handler.setFormatter(
                Formatter(
                    '%(asctime)s %(levelname)s %(message)s',
                    datefmt='%Y-%m-%dT%H:%M:%S',
                )
            )
            self._handler = handler
        except Exception as error:  # Logging must never prevent UI startup.
            self.error = str(error)

    @property
    def available(self):
        return self._handler is not None

    def write(self, level, message):
        """Append one line, permanently degrading to no-op after an error."""
        handler = self._handler
        if handler is None:
            return None

        level_name = str(level).upper()
        level_number = {
            'DEBUG': 10,
            'INFO': 20,
            'WARNING': 30,
            'ERROR': 40,
            'CRITICAL': 50,
        }.get(level_name, 20)
        one_line_message = str(message).replace('\r', ' ').replace('\n', ' ')
        record = LogRecord(
            name='arm_console.events',
            level=level_number,
            pathname='',
            lineno=0,
            msg=one_line_message,
            args=(),
            exc_info=None,
        )
        try:
            handler.handle(record)
            if handler.last_error is not None:
                raise handler.last_error
        except Exception as error:  # Includes write and rotation failures.
            self.error = str(error)
            self._disable_handler()
        return None

    def _disable_handler(self):
        handler, self._handler = self._handler, None
        if handler is None:
            return
        try:
            handler.close()
        except Exception:
            pass

    def close(self):
        self._disable_handler()
