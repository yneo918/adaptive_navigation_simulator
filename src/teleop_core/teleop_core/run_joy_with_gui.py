"""Joystick command node with GUI for 5-robot cluster control."""

import sys
import math
import rclpy
from PyQt6.QtWidgets import QApplication, QVBoxLayout, QLabel, QDoubleSpinBox
from PyQt6.QtCore import QTimer
from std_msgs.msg import Float32MultiArray
from pioneer_interfaces.msg import ClusterInfo

from .joy_cmd_base import JoyCmdBase
from .gui_base import StatusWindowBase
from .constants import DEFAULT_QOS


class JoyCmd5Robot(JoyCmdBase):
    """Joystick command node with 5-robot cluster parameters."""
    
    def __init__(self):
        super().__init__('joy_cmd')
        
        # 5-robot cluster specific parameters
        self.cluster_d2: float = 3.0
        self.cluster_d3: float = 3.0
        self.cluster_d4: float = 3.0
        self.cluster_d5: float = 3.0
        self.cluster_b3: float = 0.0
        self.cluster_b4: float = 0.0
        self.cluster_b5: float = 0.0
        
        self._setup_additional_publishers()
        self._setup_additional_subscriptions()
        
    def _setup_additional_publishers(self) -> None:
        """Setup additional publishers for cluster control."""
        self.pubsub.create_publisher(
            Float32MultiArray, '/cluster_params', DEFAULT_QOS)
        self.pubsub.create_publisher(
            Float32MultiArray, '/cluster_desired', DEFAULT_QOS)
            
    def _setup_additional_subscriptions(self) -> None:
        """Setup additional subscriptions."""
        self.pubsub.create_subscription(
            Float32MultiArray, '/cluster_desired', 
            self.cluster_desired_callback, DEFAULT_QOS)
        self.pubsub.create_subscription(
            ClusterInfo, '/cluster_info', 
            self.cluster_info_callback, DEFAULT_QOS)
        self.pubsub.create_subscription(
            ClusterInfo, '/sim/cluster_info', 
            self.cluster_info_callback, DEFAULT_QOS)
            
    def cluster_info_callback(self, msg: ClusterInfo) -> None:
        """Handle cluster info updates."""
        try:
            data = msg.cluster_desired.data
            if len(data) >= 15:
                self.cluster_x = data[0]
                self.cluster_y = data[1]
                self.cluster_t = data[2]
                self.cluster_d2 = data[8]
                self.cluster_d3 = data[9]
                self.cluster_d4 = data[10]
                self.cluster_d5 = data[11]
                self.cluster_b3 = data[12]
                self.cluster_b4 = data[13]
                self.cluster_b5 = data[14]
        except Exception as e:
            self.get_logger().error(f"Error processing cluster info: {e}")
            
    def _process_mode_changes(self, msg, toggle) -> None:
        """Override to add hidden command handling."""
        super()._process_mode_changes(msg, toggle)
        
        # Hidden command: A button with both triggers pressed
        if (self.angle_sel_button and toggle[self.angle_sel_button] == 1 and
            msg.axes[self.axis_dict.get("LT")] == 1 and 
            msg.axes[self.axis_dict.get("RT")] == 1):
            self.get_logger().info("[HIDDEN COMMAND] -RESET CLUSTER PARAMS-")
            reset_msg = Float32MultiArray(data=[-1.0, -1.0, -1.0])
            self.pubsub.publish('/cluster_params', reset_msg)


class StatusWindow5Robot(StatusWindowBase):
    """GUI window for 5-robot cluster control."""
    
    def __init__(self, node: JoyCmd5Robot):
        super().__init__(node, "JoyCmd Status - 5 Robot Cluster")
        
    def _add_custom_status_labels(self, layout: QVBoxLayout) -> None:
        """Add 5-robot specific status labels."""
        self.status_labels.update({
            "cluster_d2": QLabel(f"Cluster d2: {self.node.cluster_d2:.4f}"),
            "cluster_d3": QLabel(f"Cluster d3: {self.node.cluster_d3:.4f}"),
            "cluster_d4": QLabel(f"Cluster d4: {self.node.cluster_d4:.4f}"),
            "cluster_d5": QLabel(f"Cluster d5: {self.node.cluster_d5:.4f}"),
            "cluster_b3": QLabel(f"Cluster B3: {self.node.cluster_b3:.4f}"),
            "cluster_b4": QLabel(f"Cluster B4: {self.node.cluster_b4:.4f}"),
            "cluster_b5": QLabel(f"Cluster B5: {self.node.cluster_b5:.4f}"),
        })
        
        for label in ["cluster_d2", "cluster_d3", "cluster_d4", "cluster_d5", 
                      "cluster_b3", "cluster_b4", "cluster_b5"]:
            layout.addWidget(self.status_labels[label])
            
    def _add_custom_controls(self, layout: QVBoxLayout) -> None:
        """Add 5-robot specific control widgets."""
        # Distance parameters
        for i, param_name in enumerate(["d2", "d3", "d4", "d5"], 2):
            spin = QDoubleSpinBox()
            spin.setMinimum(0.0)
            spin.setMaximum(100.0)
            spin.setValue(getattr(self.node, f"cluster_{param_name}"))
            spin.setSingleStep(0.1)
            spin.valueChanged.connect(
                lambda v, name=param_name: self.update_cluster_distance(name, v))
            layout.addWidget(QLabel(f"Cluster {param_name}(m):"))
            layout.addWidget(spin)
            setattr(self, f"{param_name}_spin", spin)
            
        # Angle parameters
        for i, param_name in enumerate(["b3", "b4", "b5"], 3):
            spin = QDoubleSpinBox()
            spin.setMinimum(0.0)
            spin.setMaximum(2 * math.pi)
            spin.setValue(getattr(self.node, f"cluster_{param_name}"))
            spin.setSingleStep(0.1)
            spin.valueChanged.connect(
                lambda v, name=param_name: self.update_cluster_angle(name, v))
            layout.addWidget(QLabel(f"Cluster {param_name.upper()}(rad):"))
            layout.addWidget(spin)
            setattr(self, f"{param_name}_spin", spin)
            
    def _update_custom_status(self) -> None:
        """Update 5-robot specific status labels."""
        self.status_labels["cluster_d2"].setText(f"Cluster d2: {self.node.cluster_d2:.4f}")
        self.status_labels["cluster_d3"].setText(f"Cluster d3: {self.node.cluster_d3:.4f}")
        self.status_labels["cluster_d4"].setText(f"Cluster d4: {self.node.cluster_d4:.4f}")
        self.status_labels["cluster_d5"].setText(f"Cluster d5: {self.node.cluster_d5:.4f}")
        self.status_labels["cluster_b3"].setText(f"Cluster B3: {self.node.cluster_b3:.4f}")
        self.status_labels["cluster_b4"].setText(f"Cluster B4: {self.node.cluster_b4:.4f}")
        self.status_labels["cluster_b5"].setText(f"Cluster B5: {self.node.cluster_b5:.4f}")
        
    def update_cluster_distance(self, param_name: str, value: float) -> None:
        """Update cluster distance parameter."""
        setattr(self.node, f"cluster_{param_name}", value)
        self._publish_cluster_params()
        
    def update_cluster_angle(self, param_name: str, value: float) -> None:
        """Update cluster angle parameter."""
        setattr(self.node, f"cluster_{param_name}", value)
        self._publish_cluster_params()
        
    def _publish_cluster_params(self) -> None:
        """Publish cluster parameters."""
        msg = Float32MultiArray()
        msg.data = [
            self.node.cluster_d2, self.node.cluster_d3, 
            self.node.cluster_d4, self.node.cluster_d5,
            self.node.cluster_b3, self.node.cluster_b4, self.node.cluster_b5
        ]
        self.node.pubsub.publish('/cluster_params', msg)


def main(args=None):
    """Main entry point with GUI."""
    rclpy.init(args=args)
    
    # Create node
    node = JoyCmd5Robot()
    
    # Create Qt application
    app = QApplication(sys.argv)
    gui = StatusWindow5Robot(node)
    gui.show()
    
    # Setup ROS spinning timer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)  # 100Hz
    
    # Run application
    try:
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()