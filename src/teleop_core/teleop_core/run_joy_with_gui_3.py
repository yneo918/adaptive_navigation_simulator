"""Joystick command node with GUI for 3-robot cluster control."""

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


class JoyCmd3Robot(JoyCmdBase):
    """Joystick command node with 3-robot cluster parameters."""
    
    def __init__(self):
        super().__init__('joy_cmd')
        
        # 3-robot cluster specific parameters
        self.cluster_p: float = 10.0
        self.cluster_q: float = 10.0
        self.cluster_b: float = math.pi / 3
        
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
            if len(data) >= 9:
                self.cluster_x = data[0]
                self.cluster_y = data[1]
                self.cluster_t = data[2]
                self.cluster_p = data[6]
                self.cluster_q = data[7]
                self.cluster_b = data[8]
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


class StatusWindow3Robot(StatusWindowBase):
    """GUI window for 3-robot cluster control."""
    
    def __init__(self, node: JoyCmd3Robot):
        super().__init__(node, "JoyCmd Status - 3 Robot Cluster")
        
    def _add_custom_status_labels(self, layout: QVBoxLayout) -> None:
        """Add 3-robot specific status labels."""
        self.status_labels.update({
            "cluster_p": QLabel(f"Cluster P: {self.node.cluster_p:.4f}"),
            "cluster_q": QLabel(f"Cluster Q: {self.node.cluster_q:.4f}"),
            "cluster_b": QLabel(f"Cluster B: {self.node.cluster_b:.4f}"),
        })
        
        for label in ["cluster_p", "cluster_q", "cluster_b"]:
            layout.addWidget(self.status_labels[label])
            
    def _add_custom_controls(self, layout: QVBoxLayout) -> None:
        """Add 3-robot specific control widgets."""
        # P parameter
        self.p_spin = QDoubleSpinBox()
        self.p_spin.setMinimum(0.0)
        self.p_spin.setMaximum(100.0)
        self.p_spin.setValue(self.node.cluster_p)
        self.p_spin.setSingleStep(0.1)
        self.p_spin.valueChanged.connect(self.update_cluster_p)
        layout.addWidget(QLabel("Cluster P(m):"))
        layout.addWidget(self.p_spin)
        
        # Q parameter
        self.q_spin = QDoubleSpinBox()
        self.q_spin.setMinimum(0.0)
        self.q_spin.setMaximum(100.0)
        self.q_spin.setValue(self.node.cluster_q)
        self.q_spin.setSingleStep(0.1)
        self.q_spin.valueChanged.connect(self.update_cluster_q)
        layout.addWidget(QLabel("Cluster Q(m):"))
        layout.addWidget(self.q_spin)
        
        # B parameter (angle)
        self.b_spin = QDoubleSpinBox()
        self.b_spin.setMinimum(0.0)
        self.b_spin.setMaximum(2 * math.pi)
        self.b_spin.setValue(self.node.cluster_b)
        self.b_spin.setSingleStep(0.1)
        self.b_spin.valueChanged.connect(self.update_cluster_b)
        layout.addWidget(QLabel("Cluster B(rad):"))
        layout.addWidget(self.b_spin)
        
    def _update_custom_status(self) -> None:
        """Update 3-robot specific status labels."""
        self.status_labels["cluster_p"].setText(f"Cluster P: {self.node.cluster_p:.4f}")
        self.status_labels["cluster_q"].setText(f"Cluster Q: {self.node.cluster_q:.4f}")
        self.status_labels["cluster_b"].setText(f"Cluster B: {self.node.cluster_b:.4f}")
        
    def update_cluster_p(self, value: float) -> None:
        """Update cluster P parameter."""
        self.node.cluster_p = value
        self._publish_cluster_params()
        
    def update_cluster_q(self, value: float) -> None:
        """Update cluster Q parameter."""
        self.node.cluster_q = value
        self._publish_cluster_params()
        
    def update_cluster_b(self, value: float) -> None:
        """Update cluster B parameter."""
        self.node.cluster_b = value
        self._publish_cluster_params()
        
    def _publish_cluster_params(self) -> None:
        """Publish cluster parameters."""
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_p, self.node.cluster_q, self.node.cluster_b]
        self.node.pubsub.publish('/cluster_params', msg)


def main(args=None):
    """Main entry point with GUI."""
    rclpy.init(args=args)
    
    # Create node
    node = JoyCmd3Robot()
    
    # Create Qt application
    app = QApplication(sys.argv)
    gui = StatusWindow3Robot(node)
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