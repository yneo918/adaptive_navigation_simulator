import rclpy
import math
import numpy as np
import time
from rclpy.node import Node
from .ScalarGradient import ScalarGradient, ControlMode
from std_msgs.msg import Bool, Int16, String, Float32MultiArray, Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from gui_package.my_ros_module import PubSubManager

"""
The AN Node manages collects all relevant data from active robots and sends velocity command from the ScalarGradient class.
Will pulbish the values for sim or actual robots based on the which is active from joystick.
@params:
    robot_id_list: list of robot ids to listen for
"""

FREQ = 10 # Frequency to publish velocity commands
JOY_FREQ = FREQ
KV = 0.5  # Gain for computed velocity commands
MAX_SENSOR = 100.0  # Max expected sensor value in dBm
MIN_SENSOR = 0.0  # Min expected sensor value in dBm
DESIRED_SENSOR = -40.0  # Desired sensor value in dBm for cross-track controller
MAX_VEL_CLUSTER = 2.0

class ANNode(Node):
    def __init__(self):
        super().__init__('adaptive_navigator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p1", "p2", "p3", "p4", "p5"]),
                ('sensor_msg_name', "sensor")
            ]
        )
        params = self._parameters
        self.robot_id_list = self.get_parameter('robot_id_list').value # List of robot IDs to include in gradient calculation
        self.prefix = ''
        self.sensor_name = self.get_parameter('sensor_msg_name').value

        self.num_robots = len(self.robot_id_list)

        self.pubsub = PubSubManager(self)
        self.set_pubsub()

        self.gradient = ScalarGradient(num_robots=len(self.robot_id_list))

        self.future = [None] * len(self.robot_id_list)  # Store futures for each robot

        timer_period = 1 / FREQ 
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
        self.enable = False

        self.z = DESIRED_SENSOR  # Desired sensor value for the robot to navigate towards, in dBm

    def set_pubsub(self):
        """Set up publishers and subscribers."""
        self.pubsub.create_publisher(Twist, '/ctrl/cmd_vel', 5) #publish to cluster
        self.pubsub.create_subscription(String, '/ctrl/adaptive_mode', self.update_adaptive_mode, 1)
        self.pubsub.create_subscription(String, '/ctrl/cluster_mode', self._mode_callback, 1)
        for robot_id in self.robot_id_list:
            self.pubsub.create_subscription(
                Pose2D,
                f'{self.prefix}/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.pose_callback(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Float64,
                f'{self.prefix}/{robot_id}/{self.sensor_name}',
                lambda msg, robot_id=robot_id: self.sensor_callback(msg, robot_id),
                5)
        self.get_logger().info(f"Listening for robots: {self.robot_id_list}")

    def pose_callback(self, msg, robot_id):
        """Update the robot's position in (x, y, z) where z is height in scalar field"""
        robot_pose = [msg.x , msg.y]
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][0:2] = robot_pose

    def sensor_callback(self, msg, robot_id):
        """Update the robot's height in (x, y, z) where z is height in scalar field"""
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = self.normalize(msg.data, min_val=MIN_SENSOR, max_val=MAX_SENSOR)  # Set min and max based on expected sensor range

    def normalize(self, val, min_val=0.0, max_val=100.0):
        """Normalize value to a 0-1 scale which greatly improves gradient calculation."""
        val_clipped = max(min(val, max_val), min_val)  # Clamp to expected range
        norm = (val_clipped - min_val) / (max_val - min_val)
        return norm

    def update_adaptive_mode(self, msg):
        """Update the control mode based on incoming message."""
        for mode in ControlMode:
            if msg.data == mode.value:
                temp = self.gradient.mode
                self.gradient.mode = mode
                if temp != mode:
                    self.get_logger().info(f"Adaptive mode changed from {temp.value} to {mode.value}")

    def _mode_callback(self, msg: String):
        """Enable or disable adaptive navigation based on mode message."""
        if msg.data == "ADPTV_NAV_M":
            self.enable = True
        else:
            self.enable = False

    def publish_velocities(self):
        """Compute gradient and publish velocity commands."""
        _msg = Twist()
        if self.num_robots == 3:
            bearing = self.gradient.get_velocity(zdes=self.normalize(self.z))
            if bearing is None:
                self.get_logger().warn("No bearing calculated, skipping publish.")
                return
            else:
                self.get_logger().info(f"Publishing bearing: {bearing} ")
            _msg.linear.x = math.cos(bearing) * KV
            _msg.linear.y = math.sin(bearing) * KV
        if self.num_robots == 5:
            dz1 = self.gradient.robot_positions[3][2] - self.gradient.robot_positions[1][2]
            dz2 = self.gradient.robot_positions[4][2] - self.gradient.robot_positions[2][2]
            dz3 = self.gradient.robot_positions[1][2] - self.gradient.robot_positions[2][2]
            dz4 = self.gradient.robot_positions[3][2] - self.gradient.robot_positions[4][2]
            gain_x = self.gradient.mode.gain * (dz3+dz4)
            gain_y = self.gradient.mode.gain * (dz1+dz2)
            gain_t = self.gradient.mode.gain * (dz3-dz4)
            _msg.linear.x = self.clip(gain_x, abs_max=MAX_VEL_CLUSTER) * 1.0
            _msg.linear.y = self.clip(gain_y, abs_max=MAX_VEL_CLUSTER) * 1.0
            _msg.angular.z = self.clip(gain_t, abs_max=MAX_VEL_CLUSTER) * 0.5
        
        self.get_logger().info(f"z2 {self.gradient.robot_positions[1][2]}")
        self.get_logger().info(f"z3 {self.gradient.robot_positions[2][2]}")
        self.get_logger().info(f"z4 {self.gradient.robot_positions[3][2]}")
        self.get_logger().info(f"z5 {self.gradient.robot_positions[4][2]}")
        self.get_logger().info(f"d1 {dz1}")
        self.get_logger().info(f"d2 {dz2}")
        self.get_logger().info(f"d3 {dz3}")
        self.get_logger().info(f"d4 {dz4}")
        self.get_logger().info(f"x {self.gradient.mode.gain}*{dz3+dz4} = {gain_x}")
        self.get_logger().info(f"y {self.gradient.mode.gain}*{dz1+dz2} = {gain_y}")
        self.get_logger().info(f"t {self.gradient.mode.gain}*{dz3-dz4} = {gain_t}")
        self.get_logger().info(f"adp {self.gradient.mode.value}, {self.gradient.mode.gain}")
        self.get_logger().info(f"adp ctrl {_msg.linear.x}, {_msg.linear.y}, {_msg.angular.z}")

        self.pubsub.publish('/ctrl/cmd_vel', _msg) #publish velocity command to cluster
    
    def clip(self, val, abs_max=1.0, gain=10.0):
        return max(min(val*gain, abs_max), -abs_max)

    def publish_velocities_manager(self):
        """ Manage which velocities to publish based on the current output mode."""
        if self.enable:
            self.publish_velocities()

def main(args=None):
    rclpy.init(args=args)
    node = ANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()