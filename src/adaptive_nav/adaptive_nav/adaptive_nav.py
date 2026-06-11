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

FREQ = 10.0  # Frequency to publish velocity commands
MAX_FREQ = 10000.0  # Maximum frequency for time scaling (supports time_scale up to 1000)
MIN_TIME_SCALE = 0.01
MAX_TIME_SCALE = 1000.0
JOY_FREQ = FREQ
KV = 0.5  # Gain for computed velocity commands
MAX_SENSOR = 100.0  # Max expected sensor value in dBm
MIN_SENSOR = 0.0  # Min expected sensor value in dBm
DESIRED_SENSOR = 40.0  # Desired sensor value in dBm for cross-track controller
MAX_VEL_CLUSTER = 0.1
MAX_VEL_ROT_CLUSTER = 0.1
Z_DES = 0.6

# Stuck-escape (false-convergence recovery): when the experiment detects a
# convergence/plateau, the orchestrator asks the cluster to rotate about its
# centroid by a fixed angle. This re-samples the field and breaks the
# multi-peak capture where 3-4 sample points each latch onto a different peak.
# omega is derived from the per-robot speed budget so no robot exceeds its
# limit during the rotation: omega*(r_max + 1) <= ROBOT_MAX_VEL.
ESCAPE_KAPPA = 0.8       # safety margin on the per-robot speed budget
ROBOT_MAX_VEL = 10.0     # per-robot |linear|+|angular| cap (controller MAX_VEL)

class ANNode(Node):
    def __init__(self):
        super().__init__('adaptive_navigator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p1", "p2", "p3", "p4", "p5"]),
                ('sensor_msg_name', "sensor"),
                ('time_scale', 1.0),
                ('random_walk_period_steps', 50),
                ('random_walk_seed', 0),
                # Cluster translation speed [sim units/s]. Default keeps the
                # legacy stack behaviour (0.1 = 2 m/s real); the pure_sim
                # campaigns use 0.3 -- set via launch for validation runs.
                ('max_vel_cluster', MAX_VEL_CLUSTER),
            ]
        )
        params = self._parameters
        self.robot_id_list = self.get_parameter('robot_id_list').value  # List of robot IDs to include in gradient calculation
        self.prefix = ''
        self.sensor_name = self.get_parameter('sensor_msg_name').value
        self.time_scale = float(self.get_parameter('time_scale').value)
        self.max_vel_cluster = float(
            self.get_parameter('max_vel_cluster').value)

        # Validate time_scale range
        if not (MIN_TIME_SCALE <= self.time_scale <= MAX_TIME_SCALE):
            self.get_logger().error(
                f'Adaptive Nav: Invalid time_scale={self.time_scale}. '
                f'Must be between {MIN_TIME_SCALE} and {MAX_TIME_SCALE}'
            )
            raise ValueError(f'time_scale out of range: {self.time_scale}')

        # Warn if time_scale is very high
        if self.time_scale > 100.0:
            effective_freq = min(FREQ * self.time_scale, MAX_FREQ)
            self.get_logger().warn(
                f'⚠ Adaptive Nav: time_scale={self.time_scale} is very high. '
                f'Update frequency: {effective_freq:.0f} Hz'
            )

        self.num_robots = len(self.robot_id_list)
        self.goal = None

        self.pubsub = PubSubManager(self)
        self.set_pubsub()

        self.gradient = ScalarGradient(num_robots=len(self.robot_id_list))
        # Apply RANDOM_WALK baseline parameters from launch / ros2 param
        self.gradient.set_random_walk_params(
            period=int(self.get_parameter('random_walk_period_steps').value),
            seed=int(self.get_parameter('random_walk_seed').value),
        )
        # Re-apply if parameters are updated at runtime (e.g., by orchestrator
        # via `ros2 param set` between baseline trials)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.future = [None] * len(self.robot_id_list)  # Store futures for each robot

        # Adjust timer frequency based on time_scale for smooth simulation
        # Cap at MAX_FREQ to prevent unrealistic frequencies
        effective_freq = min(FREQ * self.time_scale, MAX_FREQ)
        timer_period = 1.0 / effective_freq
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
        self.enable = False

        self.z = DESIRED_SENSOR  # Desired sensor value for the robot to navigate towards, in dBm
        self.z_des = Z_DES  # Desired contour value for CROSSTRACK; overridable via /ctrl/z_des

        # Stuck-escape FSM state (driven by /ctrl/an_escape)
        self._escaping = False
        self._escape_target = 0.0       # target |Delta theta| [rad]
        self._escape_accum = 0.0        # accumulated |Delta theta| since entry [rad]
        self._escape_prev_theta = 0.0   # last measured cluster heading [rad]
        self._escape_omega = 0.0        # rotation rate [rad/s], CCW (+)

    def set_pubsub(self):
        """Set up publishers and subscribers."""
        self.pubsub.create_publisher(Twist, '/ctrl/cmd_vel', 5) #publish to cluster
        self.pubsub.create_subscription(String, '/ctrl/adaptive_mode', self.update_adaptive_mode, 1)
        self.pubsub.create_subscription(String, '/ctrl/cluster_mode', self._mode_callback, 1)
        self.pubsub.create_subscription(Float64, '/ctrl/z_des', self._z_des_callback, 1)
        # Escape command: positive value = target rotation angle [rad]; <=0 cancels.
        self.pubsub.create_subscription(Float64, '/ctrl/an_escape', self._escape_callback, 1)
        # Escape status: True while a rotation maneuver is in progress.
        self.pubsub.create_publisher(Bool, '/ctrl/an_escape_active', 1)
        self.pubsub.create_subscription(Pose2D, '/rviz/goal_pose2D', self._goal_callback, 10)
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
        self.get_logger().debug(f"Listening for robots: {self.robot_id_list}")

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
                    self.get_logger().debug(f"Adaptive mode changed from {temp.value} to {mode.value}")

    def _on_param_change(self, params):
        """Apply runtime parameter changes to ScalarGradient state.

        Used by the baseline orchestrator to switch random_walk_seed between
        trials without restarting the node.
        """
        from rcl_interfaces.msg import SetParametersResult
        need_reseed = False
        for p in params:
            if p.name == 'max_vel_cluster':
                self.max_vel_cluster = float(p.value)
            if p.name in ('random_walk_period_steps', 'random_walk_seed'):
                need_reseed = True
        if need_reseed:
            # Read current values (the new ones get applied after this callback
            # returns successful, so use param p.value if it's the changed one)
            new_period = int(self.get_parameter('random_walk_period_steps').value)
            new_seed = int(self.get_parameter('random_walk_seed').value)
            for p in params:
                if p.name == 'random_walk_period_steps':
                    new_period = int(p.value)
                elif p.name == 'random_walk_seed':
                    new_seed = int(p.value)
            self.gradient.set_random_walk_params(period=new_period, seed=new_seed)
            self.get_logger().info(
                f'RANDOM_WALK reconfigured: period={new_period}, seed={new_seed}'
            )
        return SetParametersResult(successful=True)

    def _z_des_callback(self, msg: Float64):
        """Update CROSSTRACK target contour value from external publisher."""
        new_val = float(msg.data)
        if abs(new_val - self.z_des) > 1e-6:
            self.get_logger().info(f"z_des updated: {self.z_des:.4f} -> {new_val:.4f}")
            self.z_des = new_val

    def _mode_callback(self, msg: String):
        """Enable or disable adaptive navigation based on mode message."""
        if msg.data == "ADPTV_NAV_M":
            self.enable = True
        else:
            self.enable = False
            self._escaping = False  # never rotate while navigation is disabled

    def _escape_callback(self, msg: Float64):
        """Start (or cancel) a stuck-escape rotation about the cluster centroid.

        msg.data > 0: target rotation angle [rad] (relative to the heading at
        entry). msg.data <= 0: cancel any rotation in progress.
        """
        target = float(msg.data)
        if target > 0.0:
            if self.enable and not self._escaping:
                self._begin_escape(target)
        else:
            if self._escaping:
                self.get_logger().info("ESCAPE cancelled by command")
            self._escaping = False

    def _begin_escape(self, target_angle):
        """Latch escape state and derive the rotation rate from robot speed limits."""
        centroid = self._cluster_centroid()
        r_max = max(
            float(np.linalg.norm(self._get_robot_position_2d(i) - centroid))
            for i in range(self.num_robots)
        )
        # Per-robot budget |linear| + |angular| <= ROBOT_MAX_VEL, with the
        # farthest robot at linear = omega*r_max and angular = omega (the body
        # spins with the formation). Solve for omega and apply a safety margin.
        self._escape_omega = ESCAPE_KAPPA * ROBOT_MAX_VEL / (r_max + 1.0)
        self._escape_target = float(target_angle)
        self._escape_accum = 0.0
        self._escape_prev_theta = self._cluster_heading()
        self._escaping = True
        self.get_logger().info(
            f"ESCAPE start: target={target_angle:.3f} rad, r_max={r_max:.2f}, "
            f"omega={self._escape_omega:.4f} rad/s")

    def _cluster_centroid(self):
        """Geometric centroid (x, y) of all robots."""
        pts = np.array([self._get_robot_position_2d(i)
                        for i in range(self.num_robots)])
        return pts.mean(axis=0)

    def _cluster_heading(self):
        """Cluster heading [rad] matching PentagonLeaderConfig fkine:
        theta_c = atan2(r2 - r1) - pi/2."""
        r1 = self._get_robot_position_2d(0)
        r2 = self._get_robot_position_2d(1)
        return math.atan2(r2[1] - r1[1], r2[0] - r1[0]) - math.pi / 2

    @staticmethod
    def _wrap_pi(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _compute_escape_cmd(self):
        """Rigid rotation of the formation about its centroid at omega (CCW).

        The cluster origin O (= leader p1) must orbit the centroid C, so command
        v_O(world) = omega * z_hat x (O - C), expressed in the cluster body frame
        (the controller rotates body->world by R(theta_c)). Completion is judged
        on the measured heading change (closed loop), not commanded time.
        """
        O = self._get_robot_position_2d(0)
        C = self._cluster_centroid()
        theta = self._cluster_heading()
        w = self._escape_omega

        dx, dy = O[0] - C[0], O[1] - C[1]
        vox, voy = -w * dy, w * dx           # omega * z_hat x (O - C), world frame
        ct, st = math.cos(theta), math.sin(theta)
        cmd_vel = Twist()
        cmd_vel.linear.x = ct * vox + st * voy   # R(-theta) -> body frame
        cmd_vel.linear.y = -st * vox + ct * voy
        cmd_vel.angular.z = w

        # Accumulate the actual rotation achieved.
        self._escape_accum += abs(self._wrap_pi(theta - self._escape_prev_theta))
        self._escape_prev_theta = theta
        if self._escape_accum >= self._escape_target:
            self._escaping = False
            self.get_logger().info(
                f"ESCAPE done: rotated {self._escape_accum:.3f} rad")
        return cmd_vel
    
    def _goal_callback(self, msg: Pose2D):
        self.goal = [msg.x, msg.y, msg.theta]
        self.get_logger().debug(f"Get Goal Pose {self.goal}")

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
        y_unit = AB / norm_AB

        # Unit vector y (90° counterclockwise rotation)
        x_unit = np.array([y_unit[1], -y_unit[0]])

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
        bearing = self.gradient.get_velocity(zdes=self.z_des)
        if bearing is None:
            self.get_logger().warn("No bearing calculated, skipping publish.")
            return None

        self.get_logger().debug(f"Publishing bearing: {bearing}")
        self.get_logger().debug(f"Current bearing: {self.gradient.curr_bearing}")
        self.get_logger().debug(f"grad: {self.gradient.grad}")
        self.get_logger().debug(f"r1: {self.gradient.robot_positions[0]}")
        self.get_logger().debug(f"r4: {self.gradient.robot_positions[3]}")
        self.get_logger().debug(f"r5: {self.gradient.robot_positions[4]}")

        x_unit, _ = self._compute_unit_vectors_from_AB(self._get_robot_position_2d(0), self._get_robot_position_2d(1))

        # REP103: X+ forward, Y+ left, bearing=0 means moving in X+ direction
        cmd_vel = Twist()
        bearing_cluster = bearing - math.atan2(x_unit[1], x_unit[0])
        cmd_vel.linear.x = math.cos(bearing_cluster) * self.max_vel_cluster
        cmd_vel.linear.y = math.sin(bearing_cluster) * self.max_vel_cluster
        return cmd_vel

    def _compute_velocity_5_robot_mode(self):
        """
        Compute velocity commands for 5-robot mode.

        Robot configuration (pentagon formation):
            robot1: center/leader (rear)
            robot2: left-rear (from robot1)
            robot3: right-rear (from robot1)
            robot4: left-front (from robot2)
            robot5: right-front (from robot3)

        Returns:
            Twist message with computed velocities
        """
        # Get robot positions as numpy arrays (robot1..5, 1-indexed)
        pos_r1 = self._get_robot_position_2d(0)  # robot1: center (rear)
        pos_r2 = self._get_robot_position_2d(1)  # robot2: left-rear
        pos_r3 = self._get_robot_position_2d(2)  # robot3: right-rear
        pos_r4 = self._get_robot_position_2d(3)  # robot4: left-front
        pos_r5 = self._get_robot_position_2d(4)  # robot5: right-front

        # Get sensor values (z-coordinates in scalar field)
        z_r1 = self._get_robot_sensor_value(0)
        z_r2 = self._get_robot_sensor_value(1)
        z_r3 = self._get_robot_sensor_value(2)
        z_r4 = self._get_robot_sensor_value(3)
        z_r5 = self._get_robot_sensor_value(4)

        # Compute sensor value differences along cluster X-axis (forward)
        # robot2→robot4 and robot3→robot5 pairs (rear to front)
        dz_r4_r2 = z_r4 - z_r2  # left side gradient (rear to front)
        dz_r5_r3 = z_r5 - z_r3  # right side gradient (rear to front)

        # Compute sensor value differences along cluster Y-axis (lateral)
        # robot2→robot3 and robot4→robot5 pairs (left to right)
        dz_r3_r2 = z_r3 - z_r2  # rear lateral gradient (left to right)
        dz_r5_r4 = z_r5 - z_r4  # front lateral gradient (left to right)

        # Decompose vectors to compute gain components
        # Reference frame: robot1→robot2 defines the cluster heading

        # X-direction (forward) gain: use r2→r4 and r3→r5 vectors
        comp_x_r2r4, _, _, _, _ = self._decompose_vector_CD(pos_r1, pos_r2, pos_r2, pos_r4)
        comp_x_r3r5, _, _, _, _ = self._decompose_vector_CD(pos_r1, pos_r2, pos_r3, pos_r5)
        gain_x = (dz_r4_r2 / comp_x_r2r4 + dz_r5_r3 / comp_x_r3r5)
        vel_x = self.gradient.mode.direction[0] * gain_x

        # Y-direction (lateral) gain: use r2→r3 and r4→r5 vectors
        _, comp_y_r2r3, _, _, _ = self._decompose_vector_CD(pos_r1, pos_r2, pos_r2, pos_r3)
        _, comp_y_r4r5, _, _, _ = self._decompose_vector_CD(pos_r1, pos_r2, pos_r4, pos_r5)
        gain_y = (dz_r3_r2 / comp_y_r2r3 + dz_r5_r4 / comp_y_r4r5)
        vel_y = self.gradient.mode.direction[1] * gain_y

        # Angular (rotation) gain: difference between right and left gradients
        dz_left_norm = dz_r4_r2 / comp_x_r2r4   # left side (r2→r4)
        dz_right_norm = dz_r5_r3 / comp_x_r3r5  # right side (r3→r5)
        gain_angular = (dz_right_norm - dz_left_norm) * self.gradient.mode.direction[2]
        vel_angular = gain_angular * 100.0 #* (abs(comp_x_r2r4)+abs(comp_x_r3r5))

        # Create velocity command message
        cmd_vel = Twist()
        cmd_vel.angular.z = self.clip(vel_angular, abs_max=MAX_VEL_ROT_CLUSTER)

        # Normalize linear velocity to max_vel_cluster
        vel_magnitude = 5*abs(vel_x) + abs(vel_y)
        if vel_magnitude > 0:
            cmd_vel.linear.x = (self.max_vel_cluster * vel_x * 5
                                / vel_magnitude)
            cmd_vel.linear.y = self.max_vel_cluster * vel_y / vel_magnitude
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0

        # Log debug information
        self._log_5_robot_debug_info(
            z_values=(z_r1, z_r2, z_r3, z_r4, z_r5),
            dz_values=(dz_r4_r2, dz_r5_r3, dz_r3_r2, dz_r5_r4),
            gains=(gain_x, gain_y, gain_angular)
        )

        return cmd_vel

    def _log_5_robot_debug_info(self, z_values, dz_values, gains):
        """Log debug information for 5-robot mode."""
        z_r1, z_r2, z_r3, z_r4, z_r5 = z_values
        dz_r4_r2, dz_r5_r3, dz_r3_r2, dz_r5_r4 = dz_values
        gain_x, gain_y, gain_angular = gains

        self.get_logger().debug(f"z: r1={z_r1:.4f}, r2={z_r2:.4f}, r3={z_r3:.4f}, r4={z_r4:.4f}, r5={z_r5:.4f}")
        self.get_logger().debug(f"dz: r4-r2={dz_r4_r2:.4f}, r5-r3={dz_r5_r3:.4f}, r3-r2={dz_r3_r2:.4f}, r5-r4={dz_r5_r4:.4f}")
        self.get_logger().debug(f"gains: x={gain_x:.4f}, y={gain_y:.4f}, angular={gain_angular:.4f}")
        self.get_logger().debug(f"mode: {self.gradient.mode.value}")

    def publish_velocities(self):
        """Compute gradient and publish velocity commands."""
        cmd_vel = None

        # Stuck-escape overrides normal AN control: rotate about the centroid.
        if self._escaping:
            cmd_vel = self._compute_escape_cmd()
            self.pubsub.publish('/ctrl/cmd_vel', cmd_vel)
            return

        if self.gradient.mode.num_robots == 3:
            cmd_vel = self._compute_velocity_3_robot_mode()
            if cmd_vel is None:
                return
        elif self.gradient.mode.num_robots == 5:
            cmd_vel = self._compute_velocity_5_robot_mode()
        else:
            self.get_logger().warn(f"Unsupported num_robots: {self.gradient.mode.num_robots}")
            cmd_vel = Twist()

        self.get_logger().debug(f"adp ctrl {cmd_vel.linear.x}, {cmd_vel.linear.y}, {cmd_vel.angular.z}")
        self.pubsub.publish('/ctrl/cmd_vel', cmd_vel)
    
    def clip(self, val, abs_max=1.0, gain=1.0):
        return max(min(val*gain, abs_max), -abs_max)

    def publish_velocities_manager(self):
        """ Manage which velocities to publish based on the current output mode."""
        # Always advertise escape status so the orchestrator can await completion.
        self.pubsub.publish('/ctrl/an_escape_active', Bool(data=self._escaping))
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