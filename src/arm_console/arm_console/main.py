"""Application entry point for the Arm Console."""

import os
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication, QMessageBox
import rclpy
from rclpy.utilities import remove_ros_args

from arm_console.camera_bridge import CameraBridge
from arm_console.event_log import EventLogWriter
from arm_console.main_window import MainWindow
from arm_console.ros_bridge import RosBridge


EYE_CAMERA_OPTIONS = {
    'node_name': 'arm_console_eye_camera_bridge',
    'camera_label': 'EYE CAMERA',
    'driver_node_fqn': '/camera/camera',
    'rgb_topic': '/camera/color/image_raw',
    'depth_topic': '/camera/depth/image_raw',
    'min_depth_m': 0.2,
    'max_depth_m': 5.0,
}

WRIST_D405_OPTIONS = {
    'node_name': 'arm_console_wrist_d405_bridge',
    'camera_label': 'WRIST D405',
    'driver_node_fqn': '/wrist_camera/wrist_d405',
    'rgb_topic': '/wrist_camera/wrist_d405/color/image_rect_raw',
    'depth_topic': '/wrist_camera/wrist_d405/depth/image_rect_raw',
    'min_depth_m': 0.05,
    'max_depth_m': 1.0,
}


def _load_stylesheet(application):
    package_share = get_package_share_directory('arm_console')
    stylesheet_path = os.path.join(
        package_share,
        'resource',
        'dark.qss',
    )
    with open(stylesheet_path, 'r', encoding='utf-8') as stylesheet_file:
        application.setStyleSheet(stylesheet_file.read())


def _install_shutdown_signals(application, request_close):
    """Let SIGINT/SIGTERM leave Qt's native event loop cleanly."""
    previous_handlers = {}

    def handle_shutdown(_signum, _frame):
        request_close()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, handle_shutdown)

    signal_timer = QTimer(application)
    signal_timer.setInterval(100)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()
    return signal_timer, previous_handlers


def _restore_shutdown_signals(previous_handlers):
    for signum, handler in previous_handlers.items():
        signal.signal(signum, handler)


def main(args=None):
    """Run Qt in the main thread and ROS in a dedicated executor thread."""
    ros_arguments = args if args is not None else sys.argv
    rclpy.init(args=ros_arguments)
    qt_arguments = remove_ros_args(args=ros_arguments)
    application = QApplication(qt_arguments)
    application.setApplicationName('Arm Console')
    application.setOrganizationName('Chao Robotics')

    bridge = None
    eye_camera_bridge = None
    wrist_camera_bridge = None
    window = None
    event_log = None
    signal_timer = None
    previous_signal_handlers = {}
    exit_code = 1
    try:
        _load_stylesheet(application)
        event_log = EventLogWriter()
        bridge = RosBridge()
        eye_camera_bridge = CameraBridge(**EYE_CAMERA_OPTIONS)
        wrist_camera_bridge = CameraBridge(**WRIST_D405_OPTIONS)
        window = MainWindow(
            bridge,
            event_log=event_log,
            eye_camera_bridge=eye_camera_bridge,
            wrist_camera_bridge=wrist_camera_bridge,
        )
        signal_timer, previous_signal_handlers = _install_shutdown_signals(
            application,
            window.close,
        )
        bridge.start()
        eye_camera_bridge.start()
        wrist_camera_bridge.start()
        window.show()
        exit_code = application.exec_()
    except KeyboardInterrupt:
        exit_code = 0
    except Exception as error:
        print(f'Failed to start Arm Console: {error}', file=sys.stderr)
        QMessageBox.critical(
            None,
            'Arm Console startup error',
            str(error),
        )
    finally:
        if signal_timer is not None:
            signal_timer.stop()
        if window is not None:
            window.shutdown()
        if bridge is not None:
            bridge.shutdown()
        for camera_bridge in (wrist_camera_bridge, eye_camera_bridge):
            if camera_bridge is None:
                continue
            try:
                camera_bridge.shutdown()
            except Exception as error:
                print(
                    'Failed to shut down camera bridge '
                    f'{camera_bridge.node.get_name()}: {error}',
                    file=sys.stderr,
                )
        if event_log is not None:
            event_log.close()
        rclpy.try_shutdown()
        _restore_shutdown_signals(previous_signal_handlers)

    return exit_code


if __name__ == '__main__':
    main()
