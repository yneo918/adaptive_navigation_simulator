"""Base GUI components for teleop control."""

import math
from typing import Dict, Any, Optional
from PyQt6.QtWidgets import (
    QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QSpinBox, QComboBox, QCheckBox, QDoubleSpinBox, QPushButton
)
from PyQt6.QtCore import QTimer
from std_msgs.msg import Float32MultiArray

from .constants import RoverMode
from adaptive_nav.ScalarGradient import ControlMode


class StatusWindowBase(QMainWindow):
    """Base class for status display windows."""
    
    def __init__(self, node: Any, window_title: str = "JoyCmd Status"):
        super().__init__()
        self.node = node
        self.setWindowTitle(window_title)
        
        # Initialize UI components
        self.status_labels: Dict[str, QLabel] = {}
        self.mapping_labels: Dict[str, QLabel] = {}
        
        # Setup main UI
        self._setup_ui()
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(100)  # 10Hz update
        
    def _setup_ui(self) -> None:
        """Setup the main UI layout."""
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Create three columns
        left_layout = self._create_status_layout()
        center_layout = self._create_control_layout()
        
        main_layout.addLayout(left_layout)
        main_layout.addLayout(center_layout)
        
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
    def _create_status_layout(self) -> QVBoxLayout:
        """Create the status display layout."""
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Status"))
        
        # Basic status labels
        self.status_labels = {
            "selected_rover": QLabel("Selected Rover: N/A"),
            "mode": QLabel("Mode: N/A"),
            "adaptive_nav_mode": QLabel("Adaptive Nav. Mode: N/A"),
            "hardware": QLabel("Hardware: N/A"),
            "cluster_x": QLabel(f"Cluster Xc: {self.node.cluster_x:.4f}"),
            "cluster_y": QLabel(f"Cluster Yc: {self.node.cluster_y:.4f}"),
            "cluster_t": QLabel(f"Cluster Tc: {self.node.cluster_t:.4f}"),
        }
        
        for label in self.status_labels.values():
            layout.addWidget(label)
            
        # Add additional status labels in derived classes
        self._add_custom_status_labels(layout)
        
        return layout
        
    def _create_control_layout(self) -> QVBoxLayout:
        """Create the control widgets layout."""
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Modify Status"))
        
        # Rover selection
        self.rover_spin = QSpinBox()
        self.rover_spin.setMinimum(1)
        self.rover_spin.setMaximum(self.node.n_rover)
        self.rover_spin.setValue(self.node.select)
        self.rover_spin.valueChanged.connect(self.update_rover)
        layout.addWidget(QLabel("Select Rover:"))
        layout.addWidget(self.rover_spin)
        
        # Mode selection
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([mode.value for mode in RoverMode])
        self.mode_combo.setCurrentText(self.node.rover_mode.value)
        self.mode_combo.currentTextChanged.connect(self.update_mode)
        layout.addWidget(QLabel("Mode:"))
        layout.addWidget(self.mode_combo)

        # Adaptive Mode selection
        self.adaptive_mode_combo = QComboBox()
        self.adaptive_mode_combo.addItems([mode.value for mode in ControlMode])
        self.adaptive_mode_combo.currentTextChanged.connect(self.update_adaptive_nav_mode)
        layout.addWidget(QLabel("Adaptive Mode:"))
        layout.addWidget(self.adaptive_mode_combo)

        # Enable toggle
        self.enable_button = QPushButton()
        self.enable_button.setCheckable(True)
        self.enable_button.setChecked(self.node.enable)
        self._update_enable_button_text()
        self.enable_button.toggled.connect(self.toggle_enable)
        layout.addWidget(self.enable_button)

        # Cluster position controls
        self._add_cluster_position_controls(layout)
        
        # Add custom controls in derived classes
        self._add_custom_controls(layout)
        
        return layout
       
    def _add_cluster_position_controls(self, layout: QVBoxLayout) -> None:
        """Add cluster position control widgets."""
        # X position
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setMinimum(-100.0)
        self.x_spin.setMaximum(100.0)
        self.x_spin.setValue(self.node.cluster_x)
        self.x_spin.setSingleStep(0.1)
        self.x_spin.valueChanged.connect(self.update_cluster_x)
        layout.addWidget(QLabel("Cluster Xc(m):"))
        layout.addWidget(self.x_spin)
        
        # Y position
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setMinimum(-100.0)
        self.y_spin.setMaximum(100.0)
        self.y_spin.setValue(self.node.cluster_y)
        self.y_spin.setSingleStep(0.1)
        self.y_spin.valueChanged.connect(self.update_cluster_y)
        layout.addWidget(QLabel("Cluster Yc(m):"))
        layout.addWidget(self.y_spin)
        
        # Theta (orientation)
        self.t_spin = QDoubleSpinBox()
        self.t_spin.setMinimum(-10 * math.pi)
        self.t_spin.setMaximum(10 * math.pi)
        self.t_spin.setValue(self.node.cluster_t)
        self.t_spin.setSingleStep(0.1)
        self.t_spin.valueChanged.connect(self.update_cluster_t)
        layout.addWidget(QLabel("Cluster Tc(rad):"))
        layout.addWidget(self.t_spin)
        
    def _add_custom_status_labels(self, layout: QVBoxLayout) -> None:
        """Override in derived classes to add custom status labels."""
        pass
        
    def _add_custom_controls(self, layout: QVBoxLayout) -> None:
        """Override in derived classes to add custom control widgets."""
        pass
        
    def update_status(self) -> None:
        """Update status display labels."""
        self.status_labels["selected_rover"].setText(f"Selected Rover: {self.node.select}")
        self.status_labels["mode"].setText(f"Mode: {self.node.rover_mode.value}")
        self.status_labels["adaptive_nav_mode"].setText(f"Adaptive Nav. Mode: {self.node.adaptive_nav_mode.value}")
        self.status_labels["cluster_x"].setText(f"Cluster Xc: {self.node.cluster_x:.4f}")
        self.status_labels["cluster_y"].setText(f"Cluster Yc: {self.node.cluster_y:.4f}")
        self.status_labels["cluster_t"].setText(f"Cluster Tc: {self.node.cluster_t:.4f}")

        if hasattr(self, "enable_button"):
            if self.enable_button.isChecked() != self.node.enable:
                self.enable_button.blockSignals(True)
                self.enable_button.setChecked(self.node.enable)
                self.enable_button.blockSignals(False)
            self._update_enable_button_text()

        # Update custom status in derived classes
        self._update_custom_status()
        
    def _update_custom_status(self) -> None:
        """Override in derived classes to update custom status labels."""
        pass
        
    def update_rover(self, value: int) -> None:
        """Update selected rover."""
        self.node.select = value
        
    def update_mode(self, text: str) -> None:
        """Update operation mode."""
        for mode in RoverMode:
            if mode.value == text:
                self.node.rover_mode = mode
                break
    
    def update_adaptive_nav_mode(self, text: str) -> None:
        """Update adaptive navigation mode."""
        for mode in ControlMode:
            if mode.value == text:
                self.node.adaptive_nav_mode = mode
                break
        
    def update_cluster_x(self, value: float) -> None:
        """Update cluster X position."""
        self.node.cluster_x = value
        self._publish_cluster_desired()
        
    def update_cluster_y(self, value: float) -> None:
        """Update cluster Y position."""
        self.node.cluster_y = value
        self._publish_cluster_desired()
        
    def update_cluster_t(self, value: float) -> None:
        """Update cluster orientation."""
        self.node.cluster_t = value
        self._publish_cluster_desired()
        
    def _publish_cluster_desired(self) -> None:
        """Publish cluster desired position."""
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_x, self.node.cluster_y, self.node.cluster_t]
        self.node.pubsub.publish('/ctrl/cluster_desired', msg)

    def toggle_enable(self, checked: bool) -> None:
        """Toggle enable state and refresh the button label."""
        self.node.enable = checked
        self._update_enable_button_text()

    def _update_enable_button_text(self) -> None:
        """Update the enable toggle text to reflect current state."""
        state_text = "ON" if self.node.enable else "OFF"
        self.enable_button.setText(f"Enable: {state_text}")
