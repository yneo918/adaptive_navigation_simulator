"""Demultiplexer node for distributing joystick commands."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16, String
from typing import List, Optional

from .my_ros_module import PubSubManager
from .constants import (
    RoverMode, DEFAULT_QOS, MAX_VEL_TRANS, MAX_VEL_ROT,
    DEFAULT_ROBOT_ID_PREFIX
)

class Demux(Node):
    """Demultiplexer node for routing joystick commands to multiple rovers."""
    
    def __init__(self):
        super().__init__('demux')
        
        # Initialize state
        self.block = ''
        self.select = 0
        self.mode = RoverMode.NEUTRAL
        self.broadcast = False
        self.hardware = True
        self.joy_cmd: Optional[Twist] = None
        
        # Setup parameters and robot list
        self._setup_parameters()
        
        # Initialize publishers and subscribers
        self.pubsub = PubSubManager(self)
        self._setup_subscriptions()
        self._setup_publishers()
        
    def _setup_parameters(self) -> None:
        """Setup and validate ROS parameters."""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"])
            ]
        )
        
        # Log parameters
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")
            
        # Process robot ID list
        robot_id_list = self.get_parameter('robot_id_list').value
        self.get_logger().info(f"robot_id_list: {robot_id_list}")
        
        self.robot_id_list = self._process_robot_id_list(robot_id_list)
        self.n_rover = len(self.robot_id_list)
        
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")
        
    def _process_robot_id_list(self, robot_id_list: any) -> List[str]:
        """Process and validate robot ID list parameter."""
        # Default case
        if robot_id_list is None or (isinstance(robot_id_list, list) and
                                     len(robot_id_list) == 1 and
                                     robot_id_list[0] == "p0"):
            return [f"{DEFAULT_ROBOT_ID_PREFIX}{i+1}" for i in range(6)]
            
        # Integer offset case
        if isinstance(robot_id_list, int):
            return [f"{DEFAULT_ROBOT_ID_PREFIX}{i+1+robot_id_list}" for i in range(6)]
            
        # List case
        if isinstance(robot_id_list, list):
            if all(isinstance(robot_id, int) for robot_id in robot_id_list):
                return [f"{DEFAULT_ROBOT_ID_PREFIX}{robot_id}" for robot_id in robot_id_list]
            else:
                return robot_id_list
                
        self.get_logger().error("Invalid ROBOT_ID_LIST format")
        raise ValueError("Invalid ROBOT_ID_LIST format")
        
    def _setup_subscriptions(self) -> None:
        """Setup ROS subscriptions."""
        self.pubsub.create_subscription(
            Twist, '/joy/cmd_vel', self.joy_cmd_callback, DEFAULT_QOS)
        self.pubsub.create_subscription(
            Bool, '/joy/enable', self.joy_en_callback, DEFAULT_QOS)
        self.pubsub.create_subscription(
            Int16, '/select_rover', self.sel_callback, 1)
        self.pubsub.create_subscription(
            Bool, '/joy/broadcast', self.broadcast_callback, 1)
        self.pubsub.create_subscription(
            Bool, '/joy/hardware', self.hw_sim_callback, 1)
        self.pubsub.create_subscription(
            String, '/modeC', self.mode_callback, 1)
            
    def _setup_publishers(self) -> None:
        """Setup ROS publishers for all rovers."""
        # Control command publisher for NAV mode
        self.pubsub.create_publisher(
            Twist, '/ctrl/cmd_vel', DEFAULT_QOS)
            
        # Publishers for each rover (both hardware and simulation)
        for robot_id in self.robot_id_list:
            self.pubsub.create_publisher(
                Twist, f'{self.block}/{robot_id}/cmd_vel', DEFAULT_QOS)
            self.pubsub.create_publisher(
                Twist, f'/sim/{robot_id}/cmd_vel', DEFAULT_QOS)
                
    def joy_cmd_callback(self, msg: Twist) -> None:
        """Handle joystick command velocity messages."""
        # Scale velocities by limits
        self.v_t = msg.linear.y * MAX_VEL_TRANS
        self.v_r = msg.angular.z * MAX_VEL_ROT
        self.joy_cmd = msg
        
    def broadcast_callback(self, msg: Bool) -> None:
        """Handle broadcast mode toggle."""
        self.broadcast = msg.data
        
    def hw_sim_callback(self, msg: Bool) -> None:
        """Handle hardware/simulation mode toggle."""
        self.hardware = msg.data
        
    def sel_callback(self, msg: Int16) -> None:
        """Handle rover selection changes."""
        self.select = msg.data
        
    def mode_callback(self, msg: String) -> None:
        """Handle operation mode changes."""
        try:
            # Convert string to RoverMode enum
            for mode in RoverMode:
                if mode.value == msg.data:
                    self.mode = mode
                    break
        except Exception as e:
            self.get_logger().error(f"Invalid mode received: {msg.data}")
            
    def joy_en_callback(self, msg: Bool) -> None:
        """Handle joystick enable signal and route commands."""
        try:
            enabled = msg.data
            
            # Create velocity commands
            cmd_vel = Twist()
            cmd_vel.linear.x = self.v_t if enabled else 0.0
            cmd_vel.angular.z = self.v_r if enabled else 0.0
            
            empty_twist = Twist()
            
            # Determine namespace based on hardware/sim mode
            namespace = self.block if self.hardware else "/sim"
            
            # Route commands based on mode and selection
            for i, robot_id in enumerate(self.robot_id_list):
                topic = f'{namespace}/{robot_id}/cmd_vel'
                
                if self.mode == RoverMode.NEUTRAL:
                    # Neutral mode - no movement
                    self.pubsub.publish(topic, empty_twist)
                elif self.mode == RoverMode.NAVIGATION:
                    # Cluster mode - controller handles movement
                    pass
                elif self.mode == RoverMode.ADAPTIVE_NAVIGATION:
                    # Adaptive Navigation mode - controller handles movement
                    pass
                elif self.broadcast:
                    # Broadcast mode - all rovers move
                    self.pubsub.publish(topic, cmd_vel)
                elif enabled and self.select == i + 1:
                    # Single rover selected
                    self.pubsub.publish(topic, cmd_vel)
                else:
                    # Not selected - stop
                    self.pubsub.publish(topic, empty_twist)
                    
            # In navigation mode, forward raw joystick command to controller
            if self.mode == RoverMode.NAVIGATION and self.joy_cmd is not None:
                self.pubsub.publish('/ctrl/cmd_vel', self.joy_cmd)
                
        except Exception as e:
            self.get_logger().error(f"Error in joy_en_callback: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        demux = Demux()
        rclpy.spin(demux)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'demux' in locals():
            demux.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()