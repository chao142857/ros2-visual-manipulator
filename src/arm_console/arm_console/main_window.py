"""Qt main window for the six-axis arm engineering console."""

import html

from python_qt_binding.QtCore import QDateTime, Qt, QTimer
from python_qt_binding.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from arm_console.camera_panel import CameraPanel
from arm_console.gripper_control import GripperControl
from arm_console.joint_control import JointControlRow
from arm_console.motion_status import MotionStatus
from arm_console.ros_bridge import (
    live_follow_block_reason,
    live_runtime_block_reason,
    move_block_reason,
)
from arm_console.rviz_manager import RvizManager
CALIBRATED_REFERENCE_DEGREES = (0.0, 75.0, -90.0, 0.0, 0.0, 0.0)
REFERENCE_SNAP_TOLERANCE_DEG = 1.0


def target_from_current(degrees):
    """Remove small feedback noise around the calibrated reference pose."""
    target = list(
        reference if abs(value - reference) <= REFERENCE_SNAP_TOLERANCE_DEG
        else value
        for value, reference in zip(degrees, CALIBRATED_REFERENCE_DEGREES)
    )
    return tuple(target)


class MainWindow(QMainWindow):
    """Present current state, explicit targets, and safe ROS operations."""

    def __init__(
        self,
        bridge,
        event_log=None,
        camera_bridge=None,
        eye_camera_bridge=None,
        wrist_camera_bridge=None,
    ):
        super().__init__()
        if (
            camera_bridge is not None
            and eye_camera_bridge is not None
            and camera_bridge is not eye_camera_bridge
        ):
            raise ValueError(
                'camera_bridge and eye_camera_bridge refer to different '
                'bridges'
            )
        self.bridge = bridge
        self.eye_camera_bridge = (
            eye_camera_bridge
            if eye_camera_bridge is not None
            else camera_bridge
        )
        # Preserve the original public attribute for downstream integrations.
        self.camera_bridge = self.eye_camera_bridge
        self.wrist_camera_bridge = wrist_camera_bridge
        self._event_log = event_log
        self._last_current = None
        self._targets_initialized = False
        self._backend_status = None
        self._motion_status = MotionStatus(
            state='idle',
            detail='Ready for an explicit MOVE command',
        )
        self._rviz_state = 'stopped'
        self._control_mode = 'preview'
        self._live_output_inhibited = False
        self._operation_pending = {
            'move': False,
            'stop': False,
            'disable': False,
            'gripper': False,
            'live_enable': False,
            'live_disable': False,
        }

        self.setWindowTitle('Robot Arm Control')
        self.setMinimumSize(1080, 760)
        self.resize(1320, 900)

        self.rviz_manager = RvizManager(self)
        self.rviz_manager.status_changed.connect(self._rviz_status_changed)
        self.rviz_manager.log_message.connect(self.append_log)

        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(16)
        self._preview_timer.timeout.connect(self._publish_target_preview)
        self._live_heartbeat_timer = QTimer(self)
        self._live_heartbeat_timer.setInterval(100)
        self._live_heartbeat_timer.timeout.connect(
            self._publish_live_heartbeat
        )

        central = QWidget()
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(18, 16, 18, 16)
        root_layout.setSpacing(12)

        root_layout.addWidget(self._build_header())

        body = QSplitter(Qt.Horizontal)
        body.setChildrenCollapsible(False)
        body.addWidget(self._build_joint_card())
        body.addWidget(self._build_right_panel())
        body.setStretchFactor(0, 3)
        body.setStretchFactor(1, 2)
        body.setSizes([760, 480])
        root_layout.addWidget(body, 1)

        root_layout.addWidget(self._build_safety_bar())
        root_layout.addWidget(self._build_log_card())
        self.setCentralWidget(central)

        self.bridge.signals.joint_state_received.connect(
            self._joint_state_received
        )
        self.bridge.signals.backend_status_changed.connect(
            self._backend_status_changed
        )
        self.bridge.signals.motion_status_changed.connect(
            self._motion_status_changed
        )
        self.bridge.signals.operation_finished.connect(
            self._operation_finished
        )
        self.bridge.signals.log_message.connect(self.append_log)
        if self.eye_camera_bridge is not None:
            self.eye_camera_bridge.signals.log_message.connect(
                self.append_log
            )
        if self.wrist_camera_bridge is not None:
            self.wrist_camera_bridge.signals.log_message.connect(
                self.append_log
            )

        self.append_log(
            'info',
            'Arm Console started; ARM is waiting for valid feedback',
        )
        if self._event_log is not None:
            if self._event_log.available:
                self.append_log(
                    'info',
                    f'Persistent event log: {self._event_log.path}',
                )
            else:
                self.append_log(
                    'warning',
                    'Persistent event log unavailable; UI logging remains '
                    f'active ({self._event_log.error})',
                )
        self._refresh_control_enablement()

    def _build_header(self):
        frame = QFrame()
        frame.setObjectName('headerFrame')
        frame.setProperty('card', True)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(20, 14, 20, 14)

        title_column = QVBoxLayout()
        title = QLabel('Robot Arm Control')
        title.setObjectName('windowTitle')
        subtitle = QLabel('Six-axis engineering console')
        subtitle.setObjectName('subtitleLabel')
        title_column.addWidget(title)
        title_column.addWidget(subtitle)
        layout.addLayout(title_column)
        layout.addStretch(1)

        self.header_status_dot = QLabel()
        self.header_status_dot.setObjectName('statusDot')
        self.header_status_dot.setProperty('statusState', 'waiting')
        self.header_status_dot.setFixedSize(12, 12)
        self.header_status_label = QLabel('ARM: WAITING')
        self.header_status_label.setObjectName('headerStatusLabel')
        self.header_status_label.setProperty(
            'statusState',
            'waiting',
        )
        layout.addWidget(self.header_status_dot)
        layout.addSpacing(6)
        layout.addWidget(self.header_status_label)
        return frame

    def _build_joint_card(self):
        card = QFrame()
        card.setProperty('card', True)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(8)

        title = QLabel('Joint Control')
        title.setObjectName('sectionTitle')
        self.joint_mode_description = QLabel(
            'PREVIEW ONLY · Dragging changes RViz only; MOVE sends once.'
        )
        self.joint_mode_description.setObjectName('secondaryText')
        layout.addWidget(title)
        layout.addWidget(self.joint_mode_description)

        mode_row = QHBoxLayout()
        mode_row.setSpacing(8)
        mode_label = QLabel('CONTROL MODE')
        mode_label.setObjectName('columnHeading')
        self.preview_mode_button = QPushButton('PREVIEW ONLY')
        self.preview_mode_button.setObjectName('modeButton')
        self.preview_mode_button.setCheckable(True)
        self.preview_mode_button.setChecked(True)
        self.preview_mode_button.clicked.connect(
            self._preview_mode_requested
        )
        self.live_mode_button = QPushButton('LIVE FOLLOW')
        self.live_mode_button.setObjectName('liveModeButton')
        self.live_mode_button.setCheckable(True)
        self.live_mode_button.clicked.connect(self._live_mode_requested)
        mode_row.addWidget(mode_label)
        mode_row.addStretch(1)
        mode_row.addWidget(self.preview_mode_button)
        mode_row.addWidget(self.live_mode_button)
        layout.addLayout(mode_row)
        layout.addSpacing(8)

        headings = QHBoxLayout()
        headings.setSpacing(14)
        axis_heading = QLabel('AXIS')
        axis_heading.setFixedWidth(42)
        self.current_heading = QLabel('CURRENT')
        self.current_heading.setAlignment(Qt.AlignRight)
        self.current_heading.setFixedWidth(86)
        setpoint_heading = QLabel('TARGET SETPOINT')
        target_heading = QLabel('TARGET')
        target_heading.setAlignment(Qt.AlignRight)
        target_heading.setFixedWidth(104)
        for label in (
            axis_heading,
            self.current_heading,
            setpoint_heading,
            target_heading,
        ):
            label.setObjectName('columnHeading')
        headings.addWidget(axis_heading)
        headings.addWidget(self.current_heading)
        headings.addWidget(setpoint_heading, 1)
        headings.addWidget(target_heading)
        layout.addLayout(headings)

        self.joint_rows = []
        for index, (minimum, maximum) in enumerate(
            self.bridge.joint_limits_degrees
        ):
            row = JointControlRow(
                f'J{index + 1}',
                minimum,
                maximum,
            )
            row.target_changed.connect(
                lambda value, joint_index=index:
                self._schedule_target_preview(value, joint_index)
            )
            self.joint_rows.append(row)
            layout.addWidget(row)

        layout.addStretch(1)
        action_row = QHBoxLayout()
        action_row.setSpacing(10)
        self.copy_current_button = QPushButton('TARGET = CURRENT')
        self.copy_current_button.setObjectName('secondaryButton')
        self.copy_current_button.clicked.connect(self._copy_current_to_target)
        self.move_button = QPushButton('MOVE')
        self.move_button.setObjectName('moveButton')
        self.move_button.setMinimumHeight(44)
        self.move_button.clicked.connect(self._move_requested)
        action_row.addWidget(self.copy_current_button)
        action_row.addWidget(self.move_button, 1)
        layout.addLayout(action_row)
        return card

    def _build_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self._build_system_card())
        self.module_tabs = QTabWidget()
        self.module_tabs.setObjectName('moduleTabs')
        self.module_tabs.addTab(
            self._build_visualization_card(),
            'VISUALIZATION',
        )
        self.module_tabs.addTab(self._build_gripper_card(), 'GRIPPER')
        self.module_tabs.addTab(
            self._build_eye_camera_card(),
            'EYE CAMERA',
        )
        self.module_tabs.addTab(
            self._build_wrist_camera_card(),
            'WRIST D405',
        )
        layout.addWidget(self.module_tabs, 1)
        return panel

    def _build_system_card(self):
        card = QFrame()
        card.setProperty('card', True)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        title = QLabel('Arm Control Route')
        title.setObjectName('sectionTitle')
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(9)
        self.state_stream_value = self._add_status_row(
            grid,
            0,
            'Arm feedback',
            'Never received',
        )
        self.command_value = self._add_status_row(
            grid,
            1,
            'MOVE route',
            'Discovering',
        )
        self.command_ownership_value = self._add_status_row(
            grid,
            2,
            'Command ownership',
            'Discovering',
        )
        self.control_mode_value = self._add_status_row(
            grid,
            3,
            'Control mode',
            'PREVIEW ONLY',
        )
        self.live_route_value = self._add_status_row(
            grid,
            4,
            'LIVE FOLLOW route',
            'Discovering',
        )
        self.motion_value = self._add_status_row(
            grid,
            5,
            'Motion',
            'Idle',
        )
        self.stop_service_value = self._add_status_row(
            grid,
            6,
            'STOP route',
            'Discovering',
        )
        self.disable_service_value = self._add_status_row(
            grid,
            7,
            'DISABLE route',
            'Discovering',
        )
        layout.addLayout(grid)
        return card

    def _build_visualization_card(self):
        card = QFrame()
        card.setProperty('card', True)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)

        title = QLabel('Robot / RViz')
        title.setObjectName('sectionTitle')
        model = QLabel('Dummy V2 · existing URDF and mesh assets')
        model.setObjectName('secondaryText')
        layout.addWidget(title)
        layout.addWidget(model)

        status_grid = QGridLayout()
        self.rsp_value = self._add_status_row(
            status_grid,
            0,
            'State publisher',
            'Not detected',
        )
        self.rviz_value = self._add_status_row(
            status_grid,
            1,
            'RViz process',
            'Stopped',
        )
        self.preview_value = self._add_status_row(
            status_grid,
            2,
            'Target preview',
            'Waiting for RViz',
        )
        layout.addLayout(status_grid)

        buttons = QHBoxLayout()
        self.open_rviz_button = QPushButton('OPEN RVIZ')
        self.open_rviz_button.clicked.connect(self._open_rviz)
        self.close_rviz_button = QPushButton('CLOSE RVIZ')
        self.close_rviz_button.setObjectName('secondaryButton')
        self.close_rviz_button.clicked.connect(self.rviz_manager.stop)
        buttons.addWidget(self.open_rviz_button, 1)
        buttons.addWidget(self.close_rviz_button)
        layout.addLayout(buttons)
        return card

    def _build_gripper_card(self):
        card = QFrame()
        card.setProperty('card', True)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(9)

        title = QLabel('Gripper')
        title.setObjectName('sectionTitle')
        note = QLabel(
            'Target stays local until OPEN, APPLY, or CLOSE is pressed.'
        )
        note.setObjectName('secondaryText')
        layout.addWidget(title)
        layout.addWidget(note)

        grid = QGridLayout()
        self.gripper_endpoint_value = self._add_status_row(
            grid,
            0,
            'Gripper command route',
            'Discovering',
        )
        self._add_status_row(grid, 1, 'Position feedback', 'Not available')
        layout.addLayout(grid)

        open_pulse, close_pulse = self.bridge.gripper_pulses
        self.gripper_control = GripperControl(open_pulse, close_pulse)
        self.gripper_control.command_requested.connect(
            self._gripper_requested
        )
        layout.addWidget(self.gripper_control)

        return card

    def _build_eye_camera_card(self):
        self.eye_camera_panel = CameraPanel(
            self.eye_camera_bridge,
            title='Eye Camera · Astra',
            ownership_note=(
                'Subscriber only · Astra lifecycle is owned by launch or an '
                'external driver.'
            ),
        )
        # Preserve the original attribute used by existing integrations.
        self.camera_panel = self.eye_camera_panel
        return self.eye_camera_panel

    def _build_wrist_camera_card(self):
        self.wrist_camera_panel = CameraPanel(
            self.wrist_camera_bridge,
            title='Wrist Camera · Intel RealSense D405',
            ownership_note=(
                'Subscriber only · RealSense lifecycle is owned by launch or '
                'an external driver.'
            ),
        )
        return self.wrist_camera_panel

    def _build_depth_camera_card(self):
        """Backward-compatible alias for the original Astra camera card."""
        return self._build_eye_camera_card()

    def _build_safety_bar(self):
        frame = QFrame()
        frame.setProperty('card', True)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(12)

        self.stop_button = QPushButton('STOP  ·  HOLD / 抱闸')
        self.stop_button.setObjectName('stopButton')
        self.stop_button.setMinimumHeight(48)
        self.stop_button.clicked.connect(self._stop_requested)

        self.disable_button = QPushButton('DISABLE  ·  TORQUE OFF / 卸力')
        self.disable_button.setObjectName('disableButton')
        self.disable_button.setMinimumHeight(48)
        self.disable_button.clicked.connect(self._disable_requested)

        layout.addWidget(self.stop_button, 1)
        layout.addWidget(self.disable_button, 1)
        return frame

    def _build_log_card(self):
        card = QFrame()
        card.setProperty('card', True)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 12, 16, 14)
        layout.setSpacing(7)

        title = QLabel('System Log')
        title.setObjectName('sectionTitle')
        self.log_view = QTextEdit()
        self.log_view.setObjectName('logView')
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(500)
        self.log_view.setFixedHeight(150)
        layout.addWidget(title)
        layout.addWidget(self.log_view)
        return card

    @staticmethod
    def _add_status_row(grid, row, label_text, value_text):
        label = QLabel(label_text)
        label.setObjectName('statusName')
        value = QLabel(value_text)
        value.setObjectName('statusValue')
        value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(label, row, 0)
        grid.addWidget(value, row, 1)
        return value

    def _joint_state_received(self, degrees):
        self._last_current = tuple(degrees)
        for row, value in zip(self.joint_rows, degrees):
            row.set_current(value)

        if not self._targets_initialized:
            target = target_from_current(degrees)
            for row, value in zip(self.joint_rows, target):
                row.initialize_target(value)
            self._targets_initialized = True
            self.append_log(
                'info',
                'Targets initialized from the first valid joint state; '
                'no command was sent',
            )
            self._publish_target_preview()
        self._refresh_control_enablement()

    def _backend_status_changed(self, status):
        self._backend_status = status
        self._set_status_property(
            self.header_status_dot,
            'statusState',
            status.aggregate,
        )
        self._set_status_property(
            self.header_status_label,
            'statusState',
            status.aggregate,
        )
        aggregate_labels = {
            'waiting': 'WAITING',
            'recovering': 'RECOVERING',
            'ready': 'READY',
            'feedback_lost': 'FEEDBACK LOST',
            'degraded': 'DEGRADED',
        }
        self.header_status_label.setText(
            'ARM: ' + aggregate_labels.get(
                status.aggregate,
                status.aggregate.upper(),
            )
        )

        if status.state_publishers == 0:
            publisher_text = 'no publisher'
        elif status.state_publishers == 1:
            publisher_text = '1 publisher'
        else:
            publisher_text = f'{status.state_publishers} publishers'

        if status.state_stream == 'never':
            state_text = f'WAITING · {publisher_text}'
        elif status.state_stream == 'recovering':
            state_text = (
                'RECOVERING · '
                f'{status.recovery_frames} / '
                f'{status.recovery_frames_required} valid frames · '
                f'{publisher_text}'
            )
        elif status.state_stream == 'fresh':
            state_text = (
                f'STABLE · {status.state_age_sec:.1f} s old · '
                f'{publisher_text}'
            )
        else:
            state_text = (
                f'FEEDBACK LOST · {status.state_age_sec:.1f} s old · '
                f'{publisher_text}'
            )
        self.state_stream_value.setText(state_text)
        if status.command_subscribers > 1:
            self.command_value.setText(
                f'CONFLICT · {status.command_subscribers} SUBSCRIBERS'
            )
        else:
            self.command_value.setText(
                'SUBSCRIBER DETECTED'
                if status.command_ready
                else 'NO SUBSCRIBER'
            )
        if status.command_subscribers > 1:
            self.command_ownership_value.setText('CONFLICT · MULTIPLE DRIVERS')
            self._set_status_property(
                self.command_ownership_value,
                'attention',
                True,
            )
        elif status.external_command_publishers > 0:
            self.command_ownership_value.setText(
                f'CONFLICT · {status.external_command_publishers} external'
            )
            self._set_status_property(
                self.command_ownership_value,
                'attention',
                True,
            )
        else:
            self.command_ownership_value.setText('Console only')
            self._set_status_property(
                self.command_ownership_value,
                'attention',
                False,
            )
        self.stop_service_value.setText(
            'SERVICE DISCOVERED'
            if status.stop_ready
            else 'NOT DISCOVERED'
        )
        self.disable_service_value.setText(
            'SERVICE DISCOVERED'
            if status.disable_ready
            else 'NOT DISCOVERED'
        )
        self.rsp_value.setText(
            'Detected'
            if status.robot_state_publisher_ready
            else 'Not detected'
        )
        self.preview_value.setText(
            'Ready · translucent target'
            if status.preview_ready
            else 'Waiting for GUI-managed RViz'
        )
        self.gripper_endpoint_value.setText(
            'SUBSCRIBER DETECTED'
            if status.gripper_ready
            else 'NO SUBSCRIBER'
        )
        live_route_ready = (
            status.live_command_ready
            and status.live_service_ready
            and status.live_state_ready
            and status.live_heartbeat_ready
            and status.external_live_command_publishers == 0
            and status.external_live_heartbeat_publishers == 0
        )
        if (
            status.live_command_subscribers > 1
            or status.live_state_publishers > 1
            or status.live_heartbeat_subscribers > 1
        ):
            live_route_text = 'CONFLICT · MULTIPLE DRIVERS'
        elif (
            status.external_live_command_publishers > 0
            or status.external_live_heartbeat_publishers > 0
        ):
            live_route_text = 'CONFLICT · EXTERNAL PUBLISHER'
        elif status.live_armed:
            live_route_text = 'ARMED'
        elif live_route_ready:
            live_route_text = 'READY · DISARMED'
        else:
            missing = []
            if not status.live_command_ready:
                missing.append('command')
            if not status.live_service_ready:
                missing.append('service')
            if not status.live_state_ready:
                missing.append('state')
            if not status.live_heartbeat_ready:
                missing.append('heartbeat')
            live_route_text = 'MISSING · ' + '/'.join(missing)
        self.live_route_value.setText(live_route_text)
        self._set_status_property(
            self.live_route_value,
            'attention',
            status.live_armed
            or status.external_live_command_publishers > 0
            or status.external_live_heartbeat_publishers > 0
            or status.live_command_subscribers > 1
            or status.live_state_publishers > 1
            or status.live_heartbeat_subscribers > 1,
        )

        live_change_pending = (
            self._operation_pending['live_enable']
            or self._operation_pending['live_disable']
        )
        live_runtime_blocked = (
            live_runtime_block_reason(status)
            if status.live_armed
            else None
        )
        if (
            self._control_mode == 'live'
            and live_runtime_blocked is not None
        ):
            self._live_output_inhibited = True
            self._set_control_mode('preview')
            self.append_log(
                'error',
                'LIVE FOLLOW stopped locally: ' + live_runtime_blocked,
            )
        if (
            self._control_mode == 'live'
            and not status.live_armed
            and not live_change_pending
        ):
            self._set_control_mode('preview')
            self.append_log(
                'warning',
                'LIVE FOLLOW ended by the backend; returned to PREVIEW ONLY',
            )

        stale = status.state_stream != 'fresh'
        self.current_heading.setText('LAST KNOWN' if stale else 'CURRENT')
        for row in self.joint_rows:
            row.set_current_stale(stale)

        if not self.rviz_manager.is_running:
            if status.rviz_detected:
                self._rviz_state = 'external'
            elif self._rviz_state == 'external':
                self._rviz_state = 'stopped'
            self._update_rviz_text()

        self._refresh_control_enablement()

    def _refresh_control_enablement(self):
        status = self._backend_status
        live_change_pending = (
            self._operation_pending['live_enable']
            or self._operation_pending['live_disable']
        )
        safety_action_pending = (
            self._operation_pending['stop']
            or self._operation_pending['disable']
        )
        backend_ready = (
            status is not None
            and status.aggregate == 'ready'
            and status.state_stream == 'fresh'
        )
        self.move_button.setEnabled(
            self._targets_initialized
            and backend_ready
            and self._control_mode == 'preview'
            and not (status.live_armed if status is not None else False)
            and not self._operation_pending['move']
            and not live_change_pending
            and not safety_action_pending
        )
        if safety_action_pending:
            move_tooltip = 'MOVE blocked: STOP or DISABLE is pending'
        elif status is None:
            move_tooltip = 'MOVE blocked: waiting for ARM route status'
        else:
            move_tooltip = move_block_reason(status)
        self.move_button.setToolTip(
            move_tooltip or 'Publish this target once to the ARM command route'
        )
        state_fresh = (
            status is not None and status.state_stream == 'fresh'
        )
        self.copy_current_button.setEnabled(
            self._last_current is not None
            and state_fresh
            and self._control_mode == 'preview'
            and not self._operation_pending['move']
            and not live_change_pending
            and not safety_action_pending
        )
        unexpected_live = (
            status is not None
            and status.live_armed
            and self._control_mode != 'live'
        )
        target_editable = (
            self._targets_initialized
            and not self._operation_pending['move']
            and not live_change_pending
            and not unexpected_live
            and not safety_action_pending
        )
        for row in self.joint_rows:
            row.set_target_enabled(target_editable)

        if self._operation_pending['move']:
            self.move_button.setText('SENDING…')
        else:
            self.move_button.setText('MOVE')

        if safety_action_pending:
            live_blocked = 'LIVE FOLLOW blocked: STOP or DISABLE is pending'
        elif status is None:
            live_blocked = 'LIVE FOLLOW blocked: waiting for ARM route status'
        else:
            live_blocked = live_follow_block_reason(status)
        can_arm_live = (
            self._targets_initialized
            and self._last_current is not None
            and backend_ready
            and status is not None
            and not status.live_armed
            and live_blocked is None
            and not live_change_pending
            and not self._operation_pending['move']
            and not safety_action_pending
        )
        self.live_mode_button.setEnabled(
            (can_arm_live or self._control_mode == 'live')
            and not safety_action_pending
        )
        self.live_mode_button.setToolTip(
            live_blocked
            or 'Arm a guarded 10 Hz, low-speed physical follow session'
        )
        self.preview_mode_button.setEnabled(not live_change_pending)
        if self._operation_pending['live_enable']:
            self.live_mode_button.setText('ARMING…')
        elif self._operation_pending['live_disable']:
            self.live_mode_button.setText('STOPPING…')
        else:
            self.live_mode_button.setText('LIVE FOLLOW')
        self.stop_button.setEnabled(
            status is not None
            and status.stop_ready
            and not self._operation_pending['stop']
            and not self._operation_pending['disable']
        )
        self.disable_button.setEnabled(
            status is not None
            and status.disable_ready
            and not self._operation_pending['disable']
            and not self._operation_pending['stop']
        )
        self.gripper_control.set_controls_enabled(
            status is not None
            and status.gripper_ready
            and not self._operation_pending['gripper']
        )

        external_rviz = (
            status is not None
            and status.rviz_detected
            and not self.rviz_manager.is_running
        )
        self.open_rviz_button.setEnabled(
            not self.rviz_manager.is_running and not external_rviz
        )
        self.close_rviz_button.setEnabled(self.rviz_manager.is_running)

    def _set_control_mode(self, mode):
        self._control_mode = mode
        is_live = mode == 'live'
        self.preview_mode_button.blockSignals(True)
        self.live_mode_button.blockSignals(True)
        self.preview_mode_button.setChecked(not is_live)
        self.live_mode_button.setChecked(is_live)
        self.preview_mode_button.blockSignals(False)
        self.live_mode_button.blockSignals(False)
        self.control_mode_value.setText(
            'LIVE FOLLOW · PHYSICAL' if is_live else 'PREVIEW ONLY'
        )
        self._set_status_property(
            self.control_mode_value,
            'attention',
            is_live,
        )
        self.joint_mode_description.setText(
            'LIVE FOLLOW · The latest target is followed at up to 10 Hz with '
            'driver limits; STOP exits immediately.'
            if is_live
            else 'PREVIEW ONLY · Dragging changes RViz only; MOVE sends once.'
        )
        if is_live:
            self._publish_live_heartbeat()
            self._live_heartbeat_timer.start()
        else:
            self._live_heartbeat_timer.stop()
        self._refresh_control_enablement()

    def _preview_mode_requested(self):
        self._live_output_inhibited = True
        status_live = (
            self._backend_status is not None
            and self._backend_status.live_armed
        )
        if self._control_mode != 'live' and not status_live:
            self._set_control_mode('preview')
            return
        if self._operation_pending['live_disable']:
            return
        self._set_control_mode('preview')
        self._operation_pending['live_disable'] = True
        self._refresh_control_enablement()
        self.append_log('warning', 'LIVE FOLLOW disarm requested')
        self.bridge.request_live_mode(False)

    def _live_mode_requested(self):
        if (
            self._operation_pending['stop']
            or self._operation_pending['disable']
        ):
            self.live_mode_button.setChecked(False)
            self.preview_mode_button.setChecked(True)
            self.append_log(
                'warning',
                'LIVE FOLLOW blocked: STOP or DISABLE is pending',
            )
            return
        if self._control_mode == 'live':
            self.live_mode_button.setChecked(True)
            return
        status = self._backend_status
        blocked = (
            'LIVE FOLLOW blocked: waiting for ARM route status'
            if status is None
            else live_follow_block_reason(status)
        )
        if blocked is not None:
            self.live_mode_button.setChecked(False)
            self.preview_mode_button.setChecked(True)
            self.append_log('warning', blocked)
            return
        if self._last_current is None:
            self.append_log(
                'warning',
                'LIVE FOLLOW blocked: no current joint feedback',
            )
            return

        dialog = QMessageBox(self)
        dialog.setIcon(QMessageBox.Warning)
        dialog.setWindowTitle('Arm LIVE FOLLOW')
        dialog.setText('Enable physical LIVE FOLLOW control?')
        dialog.setInformativeText(
            'The six joint targets will control the physical arm at low '
            'speed. Keep clear of the robot and use STOP to exit.'
        )
        dialog.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        dialog.setDefaultButton(QMessageBox.Cancel)
        if dialog.exec_() != QMessageBox.Yes:
            self.live_mode_button.setChecked(False)
            self.preview_mode_button.setChecked(True)
            return

        baseline = tuple(self._last_current)
        for row, value in zip(self.joint_rows, baseline):
            row.initialize_target(value)
        self._publish_target_preview()
        self._live_output_inhibited = False
        self._operation_pending['live_enable'] = True
        self._refresh_control_enablement()
        self.append_log(
            'warning',
            'LIVE FOLLOW arm requested; targets reset to Current',
        )
        self.bridge.request_live_mode(True, baseline)

    def _copy_current_to_target(self):
        if self._last_current is None:
            self.append_log('warning', 'No Current joint state is available')
            return
        target = target_from_current(self._last_current)
        for row, value in zip(self.joint_rows, target):
            row.set_target(value)
        self._schedule_target_preview()
        self.append_log(
            'info',
            'Target copied from Current; no command was sent',
        )

    def _schedule_target_preview(self, value=None, joint_index=None):
        if self._targets_initialized:
            self._preview_timer.start()
            if self._control_mode == 'live' and joint_index is not None:
                target = tuple(row.target() for row in self.joint_rows)
                self.bridge.request_live_target(target)

    def _publish_live_heartbeat(self):
        if self._control_mode == 'live':
            self.bridge.request_live_heartbeat()

    def _publish_target_preview(self):
        if not self._targets_initialized:
            return
        target = tuple(row.target() for row in self.joint_rows)
        self.bridge.request_preview(target)

    def _move_requested(self):
        if (
            self._operation_pending['stop']
            or self._operation_pending['disable']
        ):
            self.append_log(
                'warning',
                'MOVE blocked: STOP or DISABLE is pending',
            )
            return
        if self._operation_pending['move']:
            self.append_log(
                'warning',
                'MOVE publish is already pending',
            )
            return
        target = tuple(row.target() for row in self.joint_rows)
        self._operation_pending['move'] = True
        self._refresh_control_enablement()
        self.append_log('info', 'MOVE requested')
        self.bridge.request_move(target)

    def _stop_requested(self):
        self._live_output_inhibited = True
        self._set_control_mode('preview')
        self._operation_pending['stop'] = True
        self._refresh_control_enablement()
        self.append_log('warning', 'STOP requested · hold / 抱闸')
        self.bridge.request_stop()

    def _disable_requested(self):
        dialog = QMessageBox(self)
        dialog.setIcon(QMessageBox.Warning)
        dialog.setWindowTitle('Confirm DISABLE')
        dialog.setText('Disable arm motor torque?')
        dialog.setInformativeText(
            'The physical arm will unload and stop actively holding position.'
        )
        dialog.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        dialog.setDefaultButton(QMessageBox.Cancel)
        if dialog.exec_() != QMessageBox.Yes:
            return

        self._live_output_inhibited = True
        self._set_control_mode('preview')
        self._operation_pending['disable'] = True
        self._refresh_control_enablement()
        self.append_log('error', 'DISABLE requested · torque off / 卸力')
        self.bridge.request_disable()

    def _gripper_requested(self, pulse_us):
        if self._operation_pending['gripper']:
            return
        self._operation_pending['gripper'] = True
        self._refresh_control_enablement()
        self.append_log(
            'info',
            f'Gripper command requested: {pulse_us} μs',
        )
        self.bridge.request_gripper(pulse_us)

    def _operation_finished(self, operation, success, message):
        if operation in self._operation_pending:
            self._operation_pending[operation] = False
        if operation == 'live_enable':
            enter_live = success and not self._live_output_inhibited
            self._set_control_mode('live' if enter_live else 'preview')
        elif operation == 'live_disable':
            self._set_control_mode('preview')
        self.append_log('info' if success else 'error', message)
        self._refresh_control_enablement()

    def _motion_status_changed(self, status):
        self._motion_status = status
        labels = {
            'idle': 'Ready',
            'commanded': 'Command sent',
            'stopped': 'Stopped · hold',
            'disabled': 'Torque off',
            'live': 'Live follow',
        }
        motion_text = labels.get(status.state, status.state)
        self.motion_value.setText(motion_text)
        self.motion_value.setToolTip(status.detail)
        self._set_status_property(
            self.motion_value,
            'attention',
            False,
        )
        self._refresh_control_enablement()

    def _open_rviz(self):
        status = self._backend_status
        rsp_ready = (
            status.robot_state_publisher_ready if status is not None else False
        )
        rviz_detected = status.rviz_detected if status is not None else False
        self.append_log('info', 'RViz start requested')
        self.rviz_manager.start(rsp_ready, rviz_detected)

    def _rviz_status_changed(self, state):
        self._rviz_state = state
        self._update_rviz_text()
        self._refresh_control_enablement()

    def _update_rviz_text(self):
        labels = {
            'stopped': 'Stopped',
            'starting': 'Starting…',
            'running': 'Running · GUI managed',
            'stopping': 'Stopping…',
            'external': 'Running · external',
        }
        self.rviz_value.setText(labels.get(self._rviz_state, self._rviz_state))

    def append_log(self, level, message):
        """Append a semantic event, excluding high-rate joint states."""
        if self._event_log is not None:
            self._event_log.write(level, message)
        colors = {
            'info': '#9fb1c5',
            'warning': '#e2b86b',
            'error': '#ef7f7f',
        }
        timestamp = QDateTime.currentDateTime().toString('HH:mm:ss')
        safe_message = html.escape(str(message))
        color = colors.get(level, colors['info'])
        self.log_view.append(
            f'<span style="color:#667585">{timestamp}</span> '
            f'<span style="color:{color}">{safe_message}</span>'
        )
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    @staticmethod
    def _set_status_property(widget, name, value):
        widget.setProperty(name, value)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def closeEvent(self, event):
        """Stop only GUI-owned visualization children before Qt exits."""
        self.shutdown()
        event.accept()

    def shutdown(self):
        """Stop processes that were explicitly started by this console."""
        self._live_heartbeat_timer.stop()
        self.eye_camera_panel.render_timer.stop()
        self.wrist_camera_panel.render_timer.stop()
        self.rviz_manager.shutdown()
