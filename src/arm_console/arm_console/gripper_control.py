"""Local target editor for the command-only gripper interface."""

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import (
    QHBoxLayout,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

GRIPPER_MIN_PULSE_US = 600
GRIPPER_MAX_PULSE_US = 1700


class GripperControl(QWidget):
    """Edit a pulse locally and emit only explicit command requests."""

    command_requested = Signal(int)

    def __init__(self, open_pulse_us, close_pulse_us, parent=None):
        super().__init__(parent)
        self.open_pulse_us = int(open_pulse_us)
        self.close_pulse_us = int(close_pulse_us)
        minimum = GRIPPER_MIN_PULSE_US
        maximum = GRIPPER_MAX_PULSE_US

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.setSingleStep(5)
        self.slider.setPageStep(50)

        self.pulse_spin = QSpinBox()
        self.pulse_spin.setRange(minimum, maximum)
        self.pulse_spin.setSingleStep(5)
        self.pulse_spin.setSuffix(' μs')
        self.pulse_spin.setAlignment(Qt.AlignRight)
        self.pulse_spin.setFixedWidth(118)

        midpoint = round((minimum + maximum) / 2)
        self.slider.setValue(midpoint)
        self.pulse_spin.setValue(midpoint)

        target_row = QHBoxLayout()
        target_row.setContentsMargins(0, 0, 0, 0)
        target_row.setSpacing(10)
        target_row.addWidget(self.slider, 1)
        target_row.addWidget(self.pulse_spin)

        self.open_button = QPushButton('OPEN')
        self.open_button.setObjectName('secondaryButton')
        self.apply_button = QPushButton('APPLY')
        self.apply_button.setObjectName('gripperApplyButton')
        self.close_button = QPushButton('CLOSE')
        self.close_button.setObjectName('secondaryButton')

        button_row = QHBoxLayout()
        button_row.setContentsMargins(0, 0, 0, 0)
        button_row.setSpacing(8)
        button_row.addWidget(self.open_button)
        button_row.addWidget(self.apply_button, 1)
        button_row.addWidget(self.close_button)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addLayout(target_row)
        layout.addLayout(button_row)

        self.slider.valueChanged.connect(self._slider_changed)
        self.pulse_spin.valueChanged.connect(self._spin_changed)
        self.open_button.clicked.connect(
            lambda: self._request(self.open_pulse_us)
        )
        self.apply_button.clicked.connect(
            lambda: self._request(self.pulse_spin.value())
        )
        self.close_button.clicked.connect(
            lambda: self._request(self.close_pulse_us)
        )

    def _slider_changed(self, pulse_us):
        if self.pulse_spin.value() == pulse_us:
            return
        self.pulse_spin.blockSignals(True)
        self.pulse_spin.setValue(pulse_us)
        self.pulse_spin.blockSignals(False)

    def _spin_changed(self, pulse_us):
        if self.slider.value() == pulse_us:
            return
        self.slider.blockSignals(True)
        self.slider.setValue(pulse_us)
        self.slider.blockSignals(False)

    def _request(self, pulse_us):
        self.slider.setValue(pulse_us)
        self.command_requested.emit(int(pulse_us))

    def set_controls_enabled(self, enabled):
        """Enable commands only when the ROS driver endpoint exists."""
        self.slider.setEnabled(enabled)
        self.pulse_spin.setEnabled(enabled)
        self.open_button.setEnabled(enabled)
        self.apply_button.setEnabled(enabled)
        self.close_button.setEnabled(enabled)

    def pulse(self):
        """Return the unsent target pulse."""
        return self.pulse_spin.value()
