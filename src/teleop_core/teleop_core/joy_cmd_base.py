"""Base class for JoyCmd implementations."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16, String, Float32MultiArray
from typing import Dict, Optional, Tuple, Any

from .my_ros_module import JoyBase
from .constants import (
    RoverMode, DEFAULT_QOS, LOW_QOS,
    DEFAULT_N_ROVER, MAX_VEL_TRANS, MAX_VEL_ROT
)
from adaptive_nav.ScalarGradient import ControlMode


class JoyCmdBase(JoyBase):
    """Base class for joystick command processing nodes."""
    
    def __init__(self, node_name: str = 'joy_cmd'):
        super().__init__(node_name)
        
        # Initialize state variables
        self.select: int = 1
        self.hw_sel: bool = False
        self.rover_mode: RoverMode = RoverMode.NEUTRAL
        self.adaptive_nav_mode: ControlMode = ControlMode.MAX
        self.selected_angle: int = 0
        self.n_rover: int = DEFAULT_N_ROVER
        
        # Cluster parameters
        self.cluster_x: float = 0.0
        self.cluster_y: float = 0.0
        self.cluster_t: float = 0.0
        
        # Control mappings (will be set by derived classes)
        self.lx_axis: Optional[int] = None
        self.ly_axis: Optional[int] = None
        self.az_axis: Optional[int] = None
        self.en_button: Optional[int] = None
        self.rover_sel_button: Optional[int] = None
        self.mode_sel_button: Optional[int] = None
        self.angle_sel_button: Optional[int] = None
        self.hw_sel_button: Optional[int] = None
        self.broadcast_button: Optional[int] = None
        
        self._setup_parameters()
        self._setup_publishers()
        self._setup_subscriptions()
        self._setup_timer()
        
    def _setup_parameters(self) -> None:
        """Declare and load ROS parameters."""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lx', "LX"),
                ('ly', "LY"),
                ('az', "RX"),
                ('en', "LB"),
                ('rover_sel', "Y"),
                ('mode_sel', "X"),
                ('angle_sel', "A"),
                ('hardware_sim_sel', "B"),
                ('broadcast', "RB"),
                ('adaptive_nav_mode_sel', "RS"),
                ('n_rover', DEFAULT_N_ROVER),
            ]
        )
        
        # Load control mappings
        self.lx_axis = self.axis_dict.get(self.get_parameter('lx').value)
        self.ly_axis = self.axis_dict.get(self.get_parameter('ly').value)
        self.az_axis = self.axis_dict.get(self.get_parameter('az').value)
        self.en_button = self.button_dict.get(self.get_parameter('en').value)
        self.rover_sel_button = self.button_dict.get(self.get_parameter('rover_sel').value)
        self.mode_sel_button = self.button_dict.get(self.get_parameter('mode_sel').value)
        self.angle_sel_button = self.button_dict.get(self.get_parameter('angle_sel').value)
        self.hw_sel_button = self.button_dict.get(self.get_parameter('hardware_sim_sel').value)
        self.broadcast_button = self.button_dict.get(self.get_parameter('broadcast').value)
        self.adaptive_nav_mode_sel_button = self.button_dict.get(self.get_parameter('adaptive_nav_mode_sel').value)
        self.n_rover = self.get_parameter('n_rover').value
        
        self._log_parameters()
        
    def _log_parameters(self) -> None:
        """Log initialization parameters."""
        self.get_logger().info(
            f"JoyCmd Node Initialized\n"
            f"lx: {self.get_parameter('lx').value}\n"
            f"ly: {self.get_parameter('ly').value}\n"
            f"az: {self.get_parameter('az').value}\n"
            f"en: {self.get_parameter('en').value}\n"
            f"rover_sel: {self.get_parameter('rover_sel').value}\n"
            f"mode_sel: {self.get_parameter('mode_sel').value}\n"
            f"angle_sel: {self.get_parameter('angle_sel').value}\n"
            f"hardware_sel: {self.get_parameter('hardware_sim_sel').value}\n"
            f"broadcast: {self.get_parameter('broadcast').value}\n"
            f"adaptive_nav_mode_sel: {self.get_parameter('adaptive_nav_mode_sel').value}\n"
            f"n_rover: {self.n_rover}"
        )
        
    def _setup_publishers(self) -> None:
        """Setup ROS publishers."""
        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', DEFAULT_QOS)
        self.pubsub.create_publisher(Bool, '/joy/enable', DEFAULT_QOS)
        self.pubsub.create_publisher(Bool, '/joy/broadcast', LOW_QOS)
        self.pubsub.create_publisher(Bool, '/joy/hardware', LOW_QOS)
        self.pubsub.create_publisher(Int16, '/select_rover', LOW_QOS)
        self.pubsub.create_publisher(String, '/modeC', LOW_QOS)
        self.pubsub.create_publisher(String, '/modeA', LOW_QOS)
        
    def _setup_subscriptions(self) -> None:
        """Setup ROS subscriptions. Override in derived classes for additional subscriptions."""
        pass
        
    def _setup_timer(self) -> None:
        """Setup periodic timer for status publishing."""
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def joy_callback(self, msg: Joy) -> None:
        """Process joystick input messages."""
        try:
            toggle = self.joy_toggle(msg)
            self._process_mode_changes(msg, toggle)
            self._process_movement(msg)
        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")

    def _process_mode_changes(self, msg: Joy, toggle: list) -> None:
        """Process button toggles for mode changes."""
        # Rover selection
        if toggle[self.rover_sel_button] == 1:
            self.select = self.select % self.n_rover + 1
            self.get_logger().info(f"Selected Rover: {self.select}")
            
        # Hardware/Simulation mode
        if toggle[self.hw_sel_button] == 1:
            self.hw_sel = not self.hw_sel
            self.get_logger().info(f"Mode: {'Hardware' if self.hw_sel else 'Simulation'}")
            
        # Operation mode
        if toggle[self.mode_sel_button] == 1:
            modes = list(RoverMode)
            current_idx = modes.index(self.rover_mode)
            self.rover_mode = modes[(current_idx + 1) % len(modes)]
            self.get_logger().info(f"Operation Mode: {self.rover_mode.value}")

        # Adaptive Navigation Mode Select
        if toggle[self.adaptive_nav_mode_sel_button] == 1:
            adaptive_nav_modes = list(ControlMode)
            current_idx = adaptive_nav_modes.index(self.adaptive_nav_mode)
            self.adaptive_nav_mode = adaptive_nav_modes[(current_idx + 1) %
                                                                 len(adaptive_nav_modes)]
            self.get_logger().info(f"Adaptive Navigation Mode: {self.adaptive_nav_mode.value}")

        # Angle selection
        if self.angle_sel_button and toggle[self.angle_sel_button] == 1:
            self.selected_angle = (self.selected_angle + 90) % 360
            self.get_logger().info(f"Selected Angle: {self.selected_angle}")
            angle_msg = Int16()
            angle_msg.data = self.selected_angle
            self.pubsub.publish('/joy/angle_sel', angle_msg)
            
        # Broadcast mode
        broadcast_msg = Bool()
        broadcast_msg.data = msg.buttons[self.broadcast_button] == 1
        self.pubsub.publish('/joy/broadcast', broadcast_msg)
        
    def _process_movement(self, msg: Joy) -> None:
        """Process joystick movement commands."""
        if msg.buttons[self.en_button] == 1:
            # Create velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = msg.axes[self.lx_axis] * MAX_VEL_TRANS
            if self.ly_axis is not None:
                cmd_vel.linear.y = msg.axes[self.ly_axis] * MAX_VEL_TRANS
            cmd_vel.angular.z = - msg.axes[self.az_axis] * MAX_VEL_ROT
            
            # Publish velocity and enable
            self.pubsub.publish('/joy/cmd_vel', cmd_vel)
            self.pubsub.publish('/joy/enable', Bool(data=True))
        else:
            # Stop movement
            self.pubsub.publish('/joy/cmd_vel', Twist())
            self.pubsub.publish('/joy/enable', Bool(data=False))

    def timer_callback(self) -> None:
        """Periodic callback to publish status information."""
        # Publish selected rover
        select_msg = Int16()
        select_msg.data = self.select
        self.pubsub.publish('/select_rover', select_msg)
        
        # Publish operation mode
        mode_msg = String()
        mode_msg.data = self.rover_mode.value
        self.pubsub.publish('/modeC', mode_msg)
        
        # Publish hardware/sim mode
        hardware_msg = Bool()
        hardware_msg.data = self.hw_sel
        self.pubsub.publish('/joy/hardware', hardware_msg)

        # Publish adaptive navigation operation mode
        adaptive_nav_mode_msg = String()
        adaptive_nav_mode_msg.data = self.adaptive_nav_mode.value
        self.pubsub.publish('/modeA', adaptive_nav_mode_msg)

    def cluster_desired_callback(self, msg: Float32MultiArray) -> None:
        """Handle cluster desired position updates."""
        if len(msg.data) >= 3:
            self.cluster_x = msg.data[0]
            self.cluster_y = msg.data[1]
            self.cluster_t = msg.data[2]