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
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = msg.data # Set min and max based on expected sensor range

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

    def _compute_gain_from_Z_and_distance(self, A, B, C, Zb, Zc, ell, eps=1e-12,
                                          clamp_segment=False, epsilon_out=0.0):
        """
        Compute gain G(A) from Z slope along BC and signed perpendicular distance from A to line BC.

        Args:
            A, B, C: (x, y) coordinates
            Zb, Zc: scalar values at B and C
            ell: decay length for perpendicular distance (must be > 0)
            eps: small value to avoid division by zero
            clamp_segment: if True, decay outside the segment [B,C]
            epsilon_out: minimal weight outside the segment (0..1)

        Returns:
            Computed gain value
        """
        Ax, Ay = A
        Bx, By = B
        Cx, Cy = C

        # Vector along BC
        vx, vy = Cx - Bx, Cy - By
        segment_length = math.hypot(vx, vy)

        if segment_length < eps:
            # Degenerate case: B and C coincide -> no well-defined direction
            return 0.0

        # Unit direction along BC
        ux, uy = vx / segment_length, vy / segment_length

        # Vector from B to A
        rx, ry = Ax - Bx, Ay - By

        # Signed perpendicular distance from A to the line through BC
        perpendicular_distance = ux * ry - uy * rx

        # 1D slope of Z along BC
        slope = (Zc - Zb) / segment_length

        # Base distance weight (Gaussian)
        weight_perp = math.exp(-(perpendicular_distance / max(ell, eps))**2)
        weight = weight_perp

        if clamp_segment:
            # Longitudinal projection along BC
            longitudinal_distance = ux * rx + uy * ry  # signed distance along BC from B
            # Segment weight: 1 inside [0, L], fades to epsilon_out outside
            is_inside = 1.0 if (0.0 <= longitudinal_distance <= segment_length) else 0.0
            weight = weight_perp * (epsilon_out + (1.0 - epsilon_out) * is_inside)

        # Final gain
        gain = slope * weight
        return gain

    def _compute_unit_vectors_from_AB(self, A, B, eps=1e-6):
        """
        Return unit vectors x (parallel to AB) and y (orthogonal, 90° counterclockwise).

        Args:
            A, B: numpy arrays representing points
            eps: small value to avoid division by zero

        Returns:
            Tuple of (x_unit, y_unit) vectors
        """
        AB = B - A
        norm_AB = np.linalg.norm(AB)

        # Avoid division by zero
        if norm_AB < eps:
            norm_AB = eps

        # Unit vector x (same direction as AB)
        x_unit = AB / norm_AB

        # Unit vector y (90° counterclockwise rotation)
        y_unit = np.array([-x_unit[1], x_unit[0]])

        return x_unit, y_unit

    def _decompose_vector_CD(self, A, B, C, D, eps=1e-6):
        """
        Decompose vector CD into components a*x + b*y based on AB direction.

        Args:
            A, B, C, D: numpy arrays representing points
            eps: small value to avoid division by zero

        Returns:
            Tuple of (a, b, CD_reconstructed, x_unit, y_unit)
        """
        # Compute unit vectors (based on AB)
        x_unit, y_unit = self._compute_unit_vectors_from_AB(A, B, eps)

        # Compute CD vector
        CD = D - C

        # Scalar components along x and y
        component_x = np.dot(CD, x_unit)
        component_y = np.dot(CD, y_unit)

        # Reconstructed vector (for verification)
        CD_reconstructed = component_x * x_unit + component_y * y_unit

        return component_x, component_y, CD_reconstructed, x_unit, y_unit

    def _get_robot_position_2d(self, robot_index):
        """Get 2D position (x, y) of a robot by index."""
        return np.array([
            self.gradient.robot_positions[robot_index][0],
            self.gradient.robot_positions[robot_index][1]
        ])

    def _get_robot_sensor_value(self, robot_index):
        """Get sensor value (z) of a robot by index."""
        return self.gradient.robot_positions[robot_index][2]

    def _compute_velocity_3_robot_mode(self):
        """
        Compute velocity commands for 3-robot mode.

        Returns:
            Twist message with computed velocities, or None if calculation fails
        """
        bearing = self.gradient.get_velocity(zdes=self.normalize(self.z))
        if bearing is None:
            self.get_logger().warn("No bearing calculated, skipping publish.")
            return None

        self.get_logger().info(f"Publishing bearing: {bearing}")

        cmd_vel = Twist()
        cmd_vel.linear.x = math.cos(bearing) * KV
        cmd_vel.linear.y = math.sin(bearing) * KV
        return cmd_vel

    def _compute_velocity_5_robot_mode(self):
        """
        Compute velocity commands for 5-robot mode.

        Returns:
            Twist message with computed velocities
        """
        # Get robot positions as numpy arrays
        pos_robot0 = self._get_robot_position_2d(0)
        pos_robot1 = self._get_robot_position_2d(1)
        pos_robot2 = self._get_robot_position_2d(2)
        pos_robot3 = self._get_robot_position_2d(3)
        pos_robot4 = self._get_robot_position_2d(4)

        # Get sensor values (z-coordinates in scalar field)
        z_robot1 = self._get_robot_sensor_value(1)
        z_robot2 = self._get_robot_sensor_value(2)
        z_robot3 = self._get_robot_sensor_value(3)
        z_robot4 = self._get_robot_sensor_value(4)

        # Compute sensor value differences
        dz_3_to_1 = z_robot3 - z_robot1
        dz_4_to_2 = z_robot4 - z_robot2
        dz_1_to_2 = z_robot1 - z_robot2
        dz_3_to_4 = z_robot3 - z_robot4

        # Decompose vectors to compute gain components
        # For x-direction gain
        comp_x_BC, _, _, _, _ = self._decompose_vector_CD(pos_robot0, pos_robot1, pos_robot1, pos_robot2)
        comp_x_DE, _, _, _, _ = self._decompose_vector_CD(pos_robot0, pos_robot1, pos_robot3, pos_robot4)
        gain_x = (dz_1_to_2 / -(comp_x_BC) + dz_3_to_4 / -(comp_x_DE))
        vel_x = self.gradient.mode.direction[0] * gain_x * 5.0

        # For y-direction gain
        _, comp_y_BD, _, _, _ = self._decompose_vector_CD(pos_robot0, pos_robot1, pos_robot1, pos_robot3)
        _, comp_y_CE, _, _, _ = self._decompose_vector_CD(pos_robot0, pos_robot1, pos_robot2, pos_robot4)
        gain_y = (dz_3_to_1 / (comp_y_BD) + dz_4_to_2 / (comp_y_CE))
        vel_y = self.gradient.mode.direction[1] * gain_y * 5.0
        auto_cruise = -1 if vel_y < 0 else 1

        '''
        # For rotational gain (cross-track error)
        gain_cross_track_1 = self._compute_gain_from_Z_and_distance(
            A=(pos_robot0[0], pos_robot0[1]),
            B=(pos_robot1[0], pos_robot1[1]),
            C=(pos_robot4[0], pos_robot4[1]),
            Zb=z_robot1,
            Zc=z_robot4,
            ell=1.0
        )
        gain_cross_track_2 = self._compute_gain_from_Z_and_distance(
            A=(pos_robot0[0], pos_robot0[1]),
            B=(pos_robot3[0], pos_robot3[1]),
            C=(pos_robot2[0], pos_robot2[1]),
            Zb=z_robot3,
            Zc=z_robot2,
            ell=1.0
        )
        gain_angular = (gain_cross_track_1 + gain_cross_track_2)
        vel_angular = self.gradient.mode.direction[2] * gain_angular * 10.0
        '''
        dz_left_normalized = dz_4_to_2 / (comp_y_CE)   # p5-p3方向
        dz_right_normalized = dz_3_to_1 / (comp_y_BD)  # p4-p2方向

        gain_angular = (dz_right_normalized - dz_left_normalized) * self.gradient.mode.direction[2]

        vel_angular = gain_angular * 40.0

        # Create velocity command message
        cmd_vel = Twist()
        cmd_vel.linear.x = self.clip(vel_x, abs_max=MAX_VEL_CLUSTER)
        cmd_vel.linear.y = self.clip(vel_y, abs_max=MAX_VEL_CLUSTER)
        cmd_vel.angular.z = self.clip(vel_angular, abs_max=MAX_VEL_CLUSTER)

        # Log debug information
        self._log_5_robot_debug_info(
            z_values=(z_robot1, z_robot2, z_robot3, z_robot4),
            dz_values=(dz_3_to_1, dz_4_to_2, dz_1_to_2, dz_3_to_4),
            gains=(gain_x, gain_y, gain_angular)
        )

        return cmd_vel

    def _log_5_robot_debug_info(self, z_values, dz_values, gains):
        """Log debug information for 5-robot mode."""
        z_robot1, z_robot2, z_robot3, z_robot4 = z_values
        dz_3_to_1, dz_4_to_2, dz_1_to_2, dz_3_to_4 = dz_values
        gain_x, gain_y, gain_angular = gains

        self.get_logger().info(f"z2 {z_robot1}")
        self.get_logger().info(f"z3 {z_robot2}")
        self.get_logger().info(f"z4 {z_robot3}")
        self.get_logger().info(f"z5 {z_robot4}")
        self.get_logger().info(f"d1 {dz_3_to_1}")
        self.get_logger().info(f"d2 {dz_4_to_2}")
        self.get_logger().info(f"d3 {dz_1_to_2}")
        self.get_logger().info(f"d4 {dz_3_to_4}")
        self.get_logger().info(f"x {self.gradient.mode.direction[0]}*{dz_1_to_2+dz_3_to_4} = {gain_x}")
        self.get_logger().info(f"y {self.gradient.mode.direction[1]}*{dz_3_to_1+dz_4_to_2} = {gain_y}")
        self.get_logger().info(f"t {self.gradient.mode.direction[2]}*{dz_4_to_2-dz_3_to_1} = {gain_angular}")
        self.get_logger().info(f"adp {self.gradient.mode.value}")

    def publish_velocities(self):
        """Compute gradient and publish velocity commands."""
        cmd_vel = None

        if self.gradient.mode.num_robots == 3:
            cmd_vel = self._compute_velocity_3_robot_mode()
            if cmd_vel is None:
                return
        elif self.gradient.mode.num_robots == 5:
            cmd_vel = self._compute_velocity_5_robot_mode()
        else:
            self.get_logger().warn(f"Unsupported num_robots: {self.gradient.mode.num_robots}")
            cmd_vel = Twist()

        self.get_logger().info(f"adp ctrl {cmd_vel.linear.x}, {cmd_vel.linear.y}, {cmd_vel.angular.z}")
        self.pubsub.publish('/ctrl/cmd_vel', cmd_vel)
    
    def clip(self, val, abs_max=1.0, gain=1.0):
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