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
MIN_SENSOR = -100.0  # Min expected sensor value in dBm
DESIRED_SENSOR = 40.0  # Desired sensor value in dBm for cross-track controller
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
        if self.gradient.mode.num_robots == 3:
            bearing = self.gradient.get_velocity(zdes=self.normalize(self.z))
            if bearing is None:
                self.get_logger().warn("No bearing calculated, skipping publish.")
                return
            else:
                self.get_logger().info(f"Publishing bearing: {bearing} ")
            _msg.linear.x = math.cos(bearing) * KV
            _msg.linear.y = math.sin(bearing) * KV
        if self.gradient.mode.num_robots == 5:
            theta = math.atan2(self.gradient.robot_positions[1][1] - self.gradient.robot_positions[0][1],
                               self.gradient.robot_positions[1][0] - self.gradient.robot_positions[0][0])
            x1 = self.gradient.robot_positions[3][0] - self.gradient.robot_positions[1][0]
            x2 = self.gradient.robot_positions[4][0] - self.gradient.robot_positions[2][0]
            x3 = self.gradient.robot_positions[1][0] - self.gradient.robot_positions[2][0]
            x4 = self.gradient.robot_positions[3][0] - self.gradient.robot_positions[4][0]
            y1 = self.gradient.robot_positions[3][1] - self.gradient.robot_positions[1][1]
            y2 = self.gradient.robot_positions[4][1] - self.gradient.robot_positions[2][1]
            y3 = self.gradient.robot_positions[1][1] - self.gradient.robot_positions[2][1]
            y4 = self.gradient.robot_positions[3][1] - self.gradient.robot_positions[4][1]
            dy1 = -x1*math.sin(theta) + y1*math.cos(theta)
            dy2 = -x2*math.sin(theta) + y2*math.cos(theta)
            dx3 = x3*math.cos(theta) + y3*math.sin(theta)
            dx4 = x4*math.cos(theta) + y4*math.sin(theta)

            def gain_from_Z_and_distance(A, B, C, Zb, Zc, ell, eps=1e-12, clamp_segment=False, epsilon_out=0.0):
                """
                Compute gain G(A) from Z slope along BC and signed perpendicular distance from A to line BC.
                A, B, C: (x, y)
                Zb, Zc: scalar values at B and C
                ell: decay length for perpendicular distance (must be > 0)
                clamp_segment: if True, decay outside the segment [B,C]
                epsilon_out: minimal weight outside the segment (0..1)
                """
                Ax, Ay = A; Bx, By = B; Cx, Cy = C
                vx, vy = Cx - Bx, Cy - By
                L = math.hypot(vx, vy)
                if L < eps:
                    # Degenerate: B and C coincide -> no well-defined direction; fall back to zero gain
                    return 0.0

                # Unit direction along BC
                ux, uy = vx / L, vy / L

                # Vector from B to A
                rx, ry = Ax - Bx, Ay - By

                # Signed perpendicular distance from A to the line through BC
                # det(u, r) = u_x * r_y - u_y * r_x
                d = ux * ry - uy * rx

                # 1D slope of Z along BC
                s = (Zc - Zb) / L

                # Base distance weight (Gaussian)
                w_perp = math.exp(- (d / max(ell, eps))**2 )

                w = w_perp

                if clamp_segment:
                    # Longitudinal projection along BC
                    t = ux * rx + uy * ry  # signed distance along BC from B
                    # Segment weight: 1 inside [0, L], fades to epsilon_out outside
                    inside = 1.0 if (0.0 <= t <= L) else 0.0
                    w = w_perp * (epsilon_out + (1.0 - epsilon_out) * inside)

                # Final gain
                G = s * w
                return G

            dz1 = self.gradient.robot_positions[3][2] - self.gradient.robot_positions[1][2]
            dz2 = self.gradient.robot_positions[4][2] - self.gradient.robot_positions[2][2]
            dz3 = self.gradient.robot_positions[1][2] - self.gradient.robot_positions[2][2]
            dz4 = self.gradient.robot_positions[3][2] - self.gradient.robot_positions[4][2]
            gain_x = self.gradient.mode.direction[0] * (dz3+dz4) / (abs(dx3)+abs(dx4)+1e-6) * 3.0
            gain_y = self.gradient.mode.direction[1] * (dz1+dz2) / (abs(dy1)+abs(dy2)+1e-6) * 3.0
            #gain_t = self.gradient.mode.direction[2] * (dz3-dz4)
            
            gain_t = self.gradient.mode.direction[2] * (
                gain_from_Z_and_distance(
                A=(self.gradient.robot_positions[0][0], self.gradient.robot_positions[0][1]),
                B=(self.gradient.robot_positions[1][0], self.gradient.robot_positions[1][1]),
                C=(self.gradient.robot_positions[3][0], self.gradient.robot_positions[3][1]),
                Zb=self.gradient.robot_positions[1][2],
                Zc=self.gradient.robot_positions[3][2],
                ell=1.0
                ) + gain_from_Z_and_distance(
                A=(self.gradient.robot_positions[0][0], self.gradient.robot_positions[0][1]),
                B=(self.gradient.robot_positions[4][0], self.gradient.robot_positions[4][1]),
                C=(self.gradient.robot_positions[2][0], self.gradient.robot_positions[2][1]),
                Zb=self.gradient.robot_positions[4][2],
                Zc=self.gradient.robot_positions[2][2],
                ell=1.0
                )
            ) * 10.0
            
            _msg.linear.x = self.clip(gain_x, abs_max=MAX_VEL_CLUSTER)
            _msg.linear.y = self.clip(gain_y, abs_max=MAX_VEL_CLUSTER)
            _msg.angular.z = self.clip(gain_t, abs_max=MAX_VEL_CLUSTER)
            self.get_logger().info(f"z2 {self.gradient.robot_positions[1][2]}")
            self.get_logger().info(f"z3 {self.gradient.robot_positions[2][2]}")
            self.get_logger().info(f"z4 {self.gradient.robot_positions[3][2]}")
            self.get_logger().info(f"z5 {self.gradient.robot_positions[4][2]}")
            self.get_logger().info(f"d1 {dz1}")
            self.get_logger().info(f"d2 {dz2}")
            self.get_logger().info(f"d3 {dz3}")
            self.get_logger().info(f"d4 {dz4}")
            self.get_logger().info(f"x {self.gradient.mode.direction[0]}*{dz3+dz4} = {gain_x}")
            self.get_logger().info(f"y {self.gradient.mode.direction[1]}*{dz1+dz2} = {gain_y}")
            self.get_logger().info(f"t {self.gradient.mode.direction[2]}*{dz3-dz4} = {gain_t}")
            self.get_logger().info(f"adp {self.gradient.mode.value}")
        self.get_logger().info(f"adp ctrl {_msg.linear.x}, {_msg.linear.y}, {_msg.angular.z}")

        self.pubsub.publish('/ctrl/cmd_vel', _msg) #publish velocity command to cluster
    
    def clip(self, val, abs_max=1.0, gain=100.0):
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