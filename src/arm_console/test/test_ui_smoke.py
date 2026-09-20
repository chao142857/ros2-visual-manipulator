from python_qt_binding.QtCore import QObject, Signal
from python_qt_binding.QtTest import QTest
from python_qt_binding.QtWidgets import QApplication, QMessageBox

from arm_console.main_window import MainWindow, target_from_current
from arm_console.motion_status import MotionStatus
from arm_console.ros_bridge import BackendStatus


class FakeSignals(QObject):
    joint_state_received = Signal(object)
    backend_status_changed = Signal(object)
    motion_status_changed = Signal(object)
    operation_finished = Signal(str, bool, str)
    log_message = Signal(str, str)


class FakeCameraSignals(QObject):
    rgb_frame = Signal(object)
    depth_frame = Signal(object)
    status_changed = Signal(object)
    log_message = Signal(str, str)


class FakeCameraBridge:
    def __init__(self):
        self.signals = FakeCameraSignals()
        self.frames = {'rgb': None, 'depth': None}

    def take_latest_frame(self, stream):
        frame = self.frames[stream]
        self.frames[stream] = None
        return frame


class FakeBridge:
    joint_limits_degrees = ((-179.9, 179.9),) * 6
    gripper_pulses = (600, 1700)

    def __init__(self):
        self.signals = FakeSignals()
        self.move_requests = []
        self.preview_requests = []
        self.gripper_requests = []
        self.live_mode_requests = []
        self.live_target_requests = []
        self.live_heartbeat_count = 0
        self.stop_request_count = 0
        self.disable_request_count = 0

    def request_move(self, target):
        self.move_requests.append(tuple(target))

    def request_stop(self):
        self.stop_request_count += 1

    def request_disable(self):
        self.disable_request_count += 1

    def request_preview(self, target):
        self.preview_requests.append(tuple(target))

    def request_gripper(self, pulse_us):
        self.gripper_requests.append(pulse_us)

    def request_live_mode(self, enabled, baseline=None):
        baseline = None if baseline is None else tuple(baseline)
        self.live_mode_requests.append((bool(enabled), baseline))

    def request_live_target(self, target):
        self.live_target_requests.append(tuple(target))

    def request_live_heartbeat(self):
        self.live_heartbeat_count += 1


def live_ready_status(armed=True):
    """Return a complete, conflict-free LIVE route status for UI tests."""
    return BackendStatus(
        aggregate='ready',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        state_publishers=1,
        state_source_ready=True,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=True,
        live_heartbeat_ready=True,
        live_armed=armed,
    )


def test_feedback_target_is_snapped_without_overwriting_j6():
    current = (11.1, 74.4, -90.1, 0.2, -0.3, 5.1)

    assert target_from_current(current) == (
        11.1,
        75.0,
        -90.0,
        0.0,
        0.0,
        5.1,
    )


def test_two_camera_tabs_have_independent_panels_logs_and_timers():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    eye = FakeCameraBridge()
    wrist = FakeCameraBridge()
    window = MainWindow(
        bridge,
        eye_camera_bridge=eye,
        wrist_camera_bridge=wrist,
    )

    tab_labels = [
        window.module_tabs.tabText(index)
        for index in range(window.module_tabs.count())
    ]
    assert 'EYE CAMERA' in tab_labels
    assert 'WRIST D405' in tab_labels
    assert window.eye_camera_panel is not window.wrist_camera_panel
    assert window.camera_panel is window.eye_camera_panel
    assert window.eye_camera_panel.title_label.text() == 'Eye Camera · Astra'
    assert window.wrist_camera_panel.title_label.text() == (
        'Wrist Camera · Intel RealSense D405'
    )
    assert window.eye_camera_panel.render_timer.isActive()
    assert window.wrist_camera_panel.render_timer.isActive()

    eye.signals.log_message.emit(
        'info',
        'EYE CAMERA · RGB and depth streams are ready',
    )
    wrist.signals.log_message.emit(
        'warning',
        'WRIST D405 · depth stream is unavailable',
    )
    application.processEvents()
    log_text = window.log_view.toPlainText()
    assert log_text.count(
        'EYE CAMERA · RGB and depth streams are ready'
    ) == 1
    assert log_text.count(
        'WRIST D405 · depth stream is unavailable'
    ) == 1

    window.shutdown()
    assert not window.eye_camera_panel.render_timer.isActive()
    assert not window.wrist_camera_panel.render_timer.isActive()
    window.close()


def test_first_state_initializes_targets_without_sending_a_move():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)

    bridge.signals.joint_state_received.emit(
        (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    )
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='ready',
            state_stream='fresh',
            state_age_sec=0.1,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
        )
    )
    application.processEvents()

    assert bridge.move_requests == []
    assert bridge.preview_requests == [
        (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    ]
    assert [row.target() for row in window.joint_rows] == [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
    ]
    assert window.move_button.isEnabled()

    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='feedback_lost',
            state_stream='stale',
            state_age_sec=1.1,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
        )
    )
    application.processEvents()

    assert not window.move_button.isEnabled()
    assert window.stop_button.isEnabled()
    assert window.disable_button.isEnabled()
    assert window.header_status_label.text() == 'ARM: FEEDBACK LOST'
    assert window.current_heading.text() == 'LAST KNOWN'
    window.close()


def test_gripper_command_requires_explicit_button_press():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)

    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='waiting',
            state_stream='never',
            state_age_sec=None,
            command_ready=False,
            external_command_publishers=0,
            stop_ready=False,
            disable_ready=False,
            robot_state_publisher_ready=False,
            rviz_detected=False,
            gripper_ready=True,
        )
    )
    application.processEvents()

    window.gripper_control.slider.setValue(1200)
    assert bridge.gripper_requests == []

    window.gripper_control.apply_button.click()
    assert bridge.gripper_requests == [1200]
    window.close()


def test_joint_drag_updates_preview_without_sending_move():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)

    bridge.signals.joint_state_received.emit((0.0,) * 6)
    application.processEvents()
    bridge.preview_requests.clear()

    window.joint_rows[0].slider.setValue(100)
    QTest.qWait(25)
    application.processEvents()

    assert bridge.preview_requests == [
        (10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ]
    assert bridge.move_requests == []
    window.close()


def test_move_is_available_immediately_after_publish_finishes():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)

    bridge.signals.joint_state_received.emit((0.0,) * 6)
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='ready',
            state_stream='fresh',
            state_age_sec=0.1,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
        )
    )
    application.processEvents()

    window.joint_rows[0].set_target(10.0)
    window.move_button.click()
    assert bridge.move_requests == [(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
    assert not window.move_button.isEnabled()

    bridge.signals.motion_status_changed.emit(
        MotionStatus(
            state='commanded',
            detail='Command published',
        )
    )
    bridge.signals.operation_finished.emit(
        'move',
        True,
        'Command published',
    )
    application.processEvents()

    assert window.move_button.isEnabled()
    assert window.joint_rows[0].target_spin.isEnabled()
    window.joint_rows[0].set_target(20.0)
    window.move_button.click()
    assert bridge.move_requests[-1] == (20.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    window.close()


def test_arm_feedback_loss_does_not_disable_independent_gripper_route():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)

    bridge.signals.joint_state_received.emit((0.0,) * 6)
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='feedback_lost',
            state_stream='stale',
            state_age_sec=1.2,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
            gripper_ready=True,
        )
    )
    application.processEvents()

    assert not window.move_button.isEnabled()
    assert window.gripper_control.apply_button.isEnabled()
    assert window.gripper_endpoint_value.text() == 'SUBSCRIBER DETECTED'
    window.close()


def test_recovering_feedback_keeps_move_locked_until_ready():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    bridge.signals.joint_state_received.emit((0.0,) * 6)

    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='recovering',
            state_stream='recovering',
            state_age_sec=0.0,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
            recovery_frames=2,
            recovery_frames_required=3,
        )
    )
    application.processEvents()
    assert not window.move_button.isEnabled()
    assert window.header_status_label.text() == 'ARM: RECOVERING'
    assert '2 / 3' in window.state_stream_value.text()
    assert '3 consecutive valid frames' in window.move_button.toolTip()

    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='ready',
            state_stream='fresh',
            state_age_sec=0.0,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
            recovery_frames=3,
            recovery_frames_required=3,
        )
    )
    application.processEvents()
    assert window.move_button.isEnabled()
    window.close()


def test_external_command_publisher_blocks_move():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)

    bridge.signals.joint_state_received.emit((0.0,) * 6)
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='degraded',
            state_stream='fresh',
            state_age_sec=0.1,
            command_ready=True,
            external_command_publishers=1,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
        )
    )
    application.processEvents()

    assert not window.move_button.isEnabled()
    assert 'CONFLICT' in window.command_ownership_value.text()
    window.close()


def test_external_live_heartbeat_publisher_blocks_live_follow():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    bridge.signals.joint_state_received.emit((0.0,) * 6)
    status = live_ready_status(armed=False)
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            **{
                **status.__dict__,
                'external_live_heartbeat_publishers': 1,
            }
        )
    )
    application.processEvents()

    assert not window.live_mode_button.isEnabled()
    assert window.live_route_value.text() == 'CONFLICT · EXTERNAL PUBLISHER'
    assert 'heartbeat publisher' in window.live_mode_button.toolTip()
    window.close()


def test_live_follow_requires_confirmation_and_resets_to_current(monkeypatch):
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    current = (1.0, 74.0, -89.0, 2.0, 3.0, 4.0)
    bridge.signals.joint_state_received.emit(current)
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='ready',
            state_stream='fresh',
            state_age_sec=0.0,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
            live_command_ready=True,
            live_service_ready=True,
            live_state_ready=True,
            live_heartbeat_ready=True,
            state_publishers=1,
        )
    )
    application.processEvents()

    window.joint_rows[0].set_target(20.0)
    monkeypatch.setattr(
        QMessageBox,
        'exec_',
        lambda _dialog: QMessageBox.Yes,
    )
    window.live_mode_button.click()

    assert bridge.live_mode_requests == [(True, current)]
    assert [row.target() for row in window.joint_rows] == list(current)
    assert bridge.move_requests == []

    bridge.signals.operation_finished.emit(
        'live_enable',
        True,
        'LIVE FOLLOW armed',
    )
    bridge.signals.backend_status_changed.emit(
        BackendStatus(
            aggregate='ready',
            state_stream='fresh',
            state_age_sec=0.0,
            command_ready=True,
            external_command_publishers=0,
            stop_ready=True,
            disable_ready=True,
            robot_state_publisher_ready=False,
            rviz_detected=False,
            live_command_ready=True,
            live_service_ready=True,
            live_state_ready=True,
            live_heartbeat_ready=True,
            live_armed=True,
            state_publishers=1,
        )
    )
    application.processEvents()

    assert window._control_mode == 'live'
    assert not window.move_button.isEnabled()
    assert bridge.live_heartbeat_count >= 1
    heartbeat_count = bridge.live_heartbeat_count
    QTest.qWait(120)
    application.processEvents()
    assert bridge.live_heartbeat_count > heartbeat_count
    window.joint_rows[0].slider.setValue(50)
    assert bridge.live_target_requests[-1] == (
        5.0,
        74.0,
        -89.0,
        2.0,
        3.0,
        4.0,
    )
    assert bridge.move_requests == []
    window.close()


def test_live_follow_backend_disarm_returns_ui_to_preview(monkeypatch):
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    bridge.signals.joint_state_received.emit((0.0,) * 6)
    ready = BackendStatus(
        aggregate='ready',
        state_stream='fresh',
        state_age_sec=0.0,
        command_ready=True,
        external_command_publishers=0,
        stop_ready=True,
        disable_ready=True,
        robot_state_publisher_ready=False,
        rviz_detected=False,
        live_command_ready=True,
        live_service_ready=True,
        live_state_ready=True,
        live_heartbeat_ready=True,
        state_publishers=1,
    )
    bridge.signals.backend_status_changed.emit(ready)
    monkeypatch.setattr(
        QMessageBox,
        'exec_',
        lambda _dialog: QMessageBox.Yes,
    )
    window.live_mode_button.click()
    bridge.signals.operation_finished.emit(
        'live_enable',
        True,
        'LIVE FOLLOW armed',
    )
    bridge.signals.backend_status_changed.emit(
        BackendStatus(**{**ready.__dict__, 'live_armed': True})
    )
    application.processEvents()
    assert window._control_mode == 'live'

    bridge.signals.backend_status_changed.emit(ready)
    application.processEvents()

    assert window._control_mode == 'preview'
    assert window.preview_mode_button.isChecked()
    assert not window.live_mode_button.isChecked()
    window.close()


def test_preview_exit_stops_gui_heartbeat_and_failure_stays_preview():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    bridge.signals.joint_state_received.emit((0.0,) * 6)
    bridge.signals.backend_status_changed.emit(live_ready_status(armed=True))
    window._set_control_mode('live')
    application.processEvents()

    window.preview_mode_button.click()
    heartbeat_count = bridge.live_heartbeat_count
    QTest.qWait(120)
    application.processEvents()

    assert window._control_mode == 'preview'
    assert not window._live_heartbeat_timer.isActive()
    assert bridge.live_heartbeat_count == heartbeat_count
    assert bridge.live_mode_requests[-1] == (False, None)

    bridge.signals.operation_finished.emit(
        'live_disable',
        False,
        'LIVE FOLLOW disarm failed',
    )
    application.processEvents()
    assert window._control_mode == 'preview'
    assert not window._live_heartbeat_timer.isActive()
    window.close()


def test_stop_request_immediately_stops_gui_heartbeat():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    bridge.signals.joint_state_received.emit((0.0,) * 6)
    bridge.signals.backend_status_changed.emit(live_ready_status(armed=True))
    window._set_control_mode('live')
    application.processEvents()

    window.stop_button.click()
    heartbeat_count = bridge.live_heartbeat_count
    QTest.qWait(120)
    application.processEvents()

    assert bridge.stop_request_count == 1
    assert window._control_mode == 'preview'
    assert not window._live_heartbeat_timer.isActive()
    assert bridge.live_heartbeat_count == heartbeat_count
    assert not window.move_button.isEnabled()
    assert not window.live_mode_button.isEnabled()
    assert not window.joint_rows[0].target_spin.isEnabled()

    bridge.signals.operation_finished.emit('stop', False, 'STOP failed')
    application.processEvents()
    assert window._control_mode == 'preview'
    assert not window._live_heartbeat_timer.isActive()
    window.close()


def test_disable_request_immediately_stops_gui_heartbeat(monkeypatch):
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = MainWindow(bridge)
    bridge.signals.joint_state_received.emit((0.0,) * 6)
    bridge.signals.backend_status_changed.emit(live_ready_status(armed=True))
    window._set_control_mode('live')
    application.processEvents()
    monkeypatch.setattr(
        QMessageBox,
        'exec_',
        lambda _dialog: QMessageBox.Yes,
    )

    window.disable_button.click()
    heartbeat_count = bridge.live_heartbeat_count
    QTest.qWait(120)
    application.processEvents()

    assert bridge.disable_request_count == 1
    assert window._control_mode == 'preview'
    assert not window._live_heartbeat_timer.isActive()
    assert bridge.live_heartbeat_count == heartbeat_count
    assert not window.move_button.isEnabled()
    assert not window.live_mode_button.isEnabled()
    assert not window.joint_rows[0].target_spin.isEnabled()

    bridge.signals.operation_finished.emit(
        'disable',
        False,
        'DISABLE failed',
    )
    application.processEvents()
    assert window._control_mode == 'preview'
    assert not window._live_heartbeat_timer.isActive()
    window.close()
