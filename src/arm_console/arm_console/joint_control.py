"""Reusable Current/Target row for one revolute arm joint."""

import math

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QSlider,
    QWidget,
)


class JointControlRow(QWidget):
    """Display one current angle and edit one unsent target angle."""

    target_changed = Signal(float)
    slider_steps_per_degree = 10

    def __init__(self, joint_label, minimum_deg, maximum_deg, parent=None):
        super().__init__(parent)
        self.minimum_deg = minimum_deg
        self.maximum_deg = maximum_deg

        self.axis_label = QLabel(joint_label)
        self.axis_label.setObjectName('jointAxisLabel')
        self.axis_label.setFixedWidth(42)

        self.current_label = QLabel('—')
        self.current_label.setObjectName('currentValueLabel')
        self.current_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.current_label.setFixedWidth(86)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(
            math.ceil(minimum_deg * self.slider_steps_per_degree),
            math.floor(maximum_deg * self.slider_steps_per_degree),
        )
        self.slider.setSingleStep(1)
        self.slider.setPageStep(50)
        self.slider.setEnabled(False)

        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(minimum_deg, maximum_deg)
        self.target_spin.setDecimals(1)
        self.target_spin.setSingleStep(0.1)
        self.target_spin.setSuffix('°')
        self.target_spin.setAlignment(Qt.AlignRight)
        self.target_spin.setFixedWidth(104)
        self.target_spin.setKeyboardTracking(False)
        self.target_spin.setEnabled(False)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 6, 0, 6)
        layout.setSpacing(14)
        layout.addWidget(self.axis_label)
        layout.addWidget(self.current_label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.target_spin)

        self.slider.valueChanged.connect(self._slider_changed)
        self.target_spin.valueChanged.connect(self._spin_changed)

    def _slider_changed(self, slider_value):
        degree_value = slider_value / self.slider_steps_per_degree
        if abs(self.target_spin.value() - degree_value) >= 0.001:
            self.target_spin.blockSignals(True)
            self.target_spin.setValue(degree_value)
            self.target_spin.blockSignals(False)
        self.target_changed.emit(degree_value)

    def _spin_changed(self, degree_value):
        slider_value = round(degree_value * self.slider_steps_per_degree)
        slider_value = max(self.slider.minimum(), slider_value)
        slider_value = min(self.slider.maximum(), slider_value)
        if self.slider.value() != slider_value:
            self.slider.blockSignals(True)
            self.slider.setValue(slider_value)
            self.slider.blockSignals(False)
        self.target_changed.emit(degree_value)

    def set_current(self, degree_value):
        """Update only the measured Current column."""
        self.current_label.setText(f'{degree_value:+.1f}°')

    def set_current_stale(self, stale):
        """Visually mark retained Current values as stale."""
        self.current_label.setProperty('stale', stale)
        self.current_label.style().unpolish(self.current_label)
        self.current_label.style().polish(self.current_label)

    def initialize_target(self, degree_value):
        """Initialize Target without emitting a movement command."""
        bounded = max(self.minimum_deg, min(self.maximum_deg, degree_value))
        self.target_spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.target_spin.setValue(bounded)
        self.slider.setValue(
            round(bounded * self.slider_steps_per_degree)
        )
        self.slider.blockSignals(False)
        self.target_spin.blockSignals(False)
        self.set_target_enabled(True)

    def set_target(self, degree_value):
        """Set an editable target without publishing it."""
        bounded = max(self.minimum_deg, min(self.maximum_deg, degree_value))
        self.target_spin.setValue(bounded)

    def set_target_enabled(self, enabled):
        """Enable target editing after a valid initial state is known."""
        self.slider.setEnabled(enabled)
        self.target_spin.setEnabled(enabled)

    def target(self):
        """Return the unsent target in degrees."""
        return self.target_spin.value()
