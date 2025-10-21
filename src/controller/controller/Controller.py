import rclpy
import math
import numpy as np
import time
from typing import Dict, List, Optional, Tuple
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool, Int16
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, Pose2D
from pioneer_interfaces.msg import ClusterInfo
from teleop_core.my_ros_module import PubSubManager
from cluster_node.Cluster import Cluster, ClusterConfig, ControlMode

# Constants
FREQ = 10
JOY_FREQ = FREQ # Frequency for joystick commands
KP_GAIN = 1.0  # Position gain in m/s per m error
KV_GAIN = 0.2  # Velocity gain in m/s per m/s error
EPSILON = 0.5  # Distance threshold to consider robot at desired position in meters
MAX_VEL = 3.0  # Maximum linear+angular velocity in m/s
ROVER_DOF = 3  # (x, y, theta)

class RobotStatus:
    """Container for robot status information"""
    def __init__(self):
        self.pose: Optional[List[float]] = None
        self.velocity: Optional[List[float]] = None
        self.sensor_data: Optional[float] = None
        self.pose_timestamp: Optional[float] = None
        self.sensor_timestamp: Optional[float] = None
    
    def update_pose(self, pose: List[float], velocity: List[float], timestamp: float):
        """Update pose and velocity information"""
        self.pose = pose
        self.velocity = velocity
        self.pose_timestamp = timestamp
    
    def is_alive(self) -> bool:
        """Check if robot is reporting data"""
        return self.pose is not None

class Controller(Node):
    """
    Main controller node for Pioneer robot cluster management.
    Handles cluster formation, navigation, and velocity commands.
    """
    
    def __init__(self, node_name: str = 'controller'):
        super().__init__(node_name)
        self._initialize_parameters()
        self._initialize_cluster_data()
        self._setup_communication()
        self._start_control_loop()

    def _initialize_parameters(self):
        """Initialize ROS parameters and validate them"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p1", "p2", "p3"]),
                ('cluster_size', 3),
                ('cluster_params', [8.0, 8.0, 1.047]), 
                ('cluster_type', "TriangleatCentroid"),
                ('control_mode', "POS"),
            ]
        )
        
        # Log all parameters
        for name, param in self._parameters.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")
        
        # Parse and validate robot IDs
        robot_id_list = self.get_parameter('robot_id_list').value
        self._setup_robot_lists(robot_id_list)
        
        # Validate cluster parameters
        self.cluster_params = self.get_parameter('cluster_params').value
        self.cluster_size = self.get_parameter('cluster_size').value
        self.cluster_type = self.get_parameter('cluster_type').value
        self.control_mode = self.get_parameter('control_mode').value
        expected_param_length = self.cluster_size * ROVER_DOF - self.cluster_size - ROVER_DOF
        if len(self.cluster_params) != expected_param_length:
            self.get_logger().error(
                f"Invalid cluster_params length: expected {expected_param_length}, "
                f"got {len(self.cluster_params)}"
            )
            raise ValueError(
                f"cluster_params must be of length {expected_param_length}, "
                f"got {len(self.cluster_params)}"
            )
        self.get_logger().info(
            f"Cluster parameters initialized: {self.cluster_params}"
        )

    def _setup_robot_lists(self, robot_id_list: List[str]):
        """Initialize robot ID lists and status dictionaries"""
        if not isinstance(robot_id_list, list):
            self.get_logger().error("robot_id_list must be a list of strings")
            raise ValueError("robot_id_list must be a list of strings")
        
        self.robot_id_list = robot_id_list
        self.n_rover = len(robot_id_list)
        
        # Initialize status dictionaries
        self.actual_robot_status: Dict[str, RobotStatus] = {}
        self.sim_robot_status: Dict[str, RobotStatus] = {}
        
        for robot_id in self.robot_id_list:
            self.actual_robot_status[robot_id] = RobotStatus()
            self.sim_robot_status[robot_id] = RobotStatus()
        
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")

    def _initialize_cluster_data(self):
        """Initialize cluster-related data structures"""        
        # Configuration flags
        self.actual_configured = False
        self.sim_configured = False
        
        # Robot lists
        self.registered_robots: List[str] = []
        self.sim_registered_robots: List[str] = []
        
        # Actual robot cluster data
        self._initialize_cluster_arrays()
        
        # Create cluster object
        try:
            self.cluster = Cluster(
                num_robots=self.cluster_size,
                cluster_type=self.cluster_type,
                kp_gains=[KP_GAIN] * (self.cluster_size * ROVER_DOF),
                kv_gains=[KV_GAIN] * (self.cluster_size * ROVER_DOF),
                control_mode=self.control_mode
            )
            self.get_logger().info(f"Cluster setup with type: {self.cluster_type}")
        except Exception as e:
            self.get_logger().error(f"Failed to create cluster: {e}")
        
        # Control state
        self.output = "actual"  # Switch between simulation and actual robots
        self.mode = "INIT"  # Control mode
        self.cluster_enable = False
        self.enable = False
        self.joy_timestamp: Optional[float] = None
        
        self.get_logger().info(f"Cluster initialized with size: {self.cluster_size}")

    def _initialize_cluster_arrays(self):
        """Initialize numpy arrays for cluster calculations"""
        array_size = (self.cluster_size * ROVER_DOF, 1)
        
        # Current and desired cluster positions
        self.c = np.zeros(array_size)
        self.c_des = np.zeros(array_size)
        self.cdot_des = np.zeros(array_size)
        
        # Robot positions and velocities
        self.r = np.zeros(array_size)
        self.rdot = np.zeros(array_size)
        
        # Simulation arrays
        self.sim_c = np.zeros(array_size)
        self.sim_cdot_des = np.zeros(array_size)
        self.sim_r = np.zeros(array_size)
        self.sim_rdot = np.zeros(array_size)
        
        # Set initial desired positions
        param_start = self.cluster_size * ROVER_DOF - len(self.cluster_params)
        param_end = self.cluster_size * ROVER_DOF
        self.c_des[param_start:param_end] = np.reshape(
            self.cluster_params, (len(self.cluster_params), 1)
        )
        
        self.time = None
        self.sim_time = None

    def _setup_communication(self):
        """Setup ROS publishers and subscribers"""
        self.pubsub = PubSubManager(self)
        
        # Global subscriptions
        self._create_global_subscriptions()
        
        # Global publishers
        self._create_global_publishers()
        
        # Robot-specific topics
        self._create_robot_topics()
        
        self.get_logger().info(f"Listening for robots: {self.robot_id_list}")

    def _create_global_subscriptions(self):
        """Create global topic subscriptions"""
        subscriptions = [
            (Bool, '/joy/hardware', self._hw_sim_callback, 1),
            (String, '/modeC', self._mode_callback, 1),
            (Twist, '/ctrl/cmd_vel', self._cmd_callback, 5),
            (Bool, '/joy/enable', self._enable_callback, 5),
            (Float32MultiArray, '/cluster_params', self._cluster_params_callback, 5),
            (Float32MultiArray, '/cluster_desired', self._cluster_desired_callback, 5),
        ]
        
        for msg_type, topic, callback, qos in subscriptions:
            self.pubsub.create_subscription(msg_type, topic, callback, qos)

    def _create_global_publishers(self):
        """Create global topic publishers"""
        publishers = [
            (ClusterInfo, '/cluster_info', 5),
            (ClusterInfo, '/sim/cluster_info', 5),
        ]
        
        for msg_type, topic, qos in publishers:
            self.pubsub.create_publisher(msg_type, topic, qos)

    def _create_robot_topics(self):
        """Create robot-specific topics"""
        for robot_id in self.robot_id_list:
            # Subscriptions
            self.pubsub.create_subscription(
                Pose2D, f'/{robot_id}/pose2D',
                lambda msg, rid=robot_id: self._robot_pose_callback(msg, rid, "actual"), 5
            )
            self.pubsub.create_subscription(
                Pose2D, f'/sim/{robot_id}/pose2D',
                lambda msg, rid=robot_id: self._robot_pose_callback(msg, rid, "sim"), 5
            )
            
            # Publishers
            topics = [
                (Pose2D, f'/{robot_id}/desiredPose2D', 5),
                (Twist, f'/{robot_id}/cmd_vel', 5),
                (Twist, f'/sim/{robot_id}/cmd_vel', 5),
            ]
            
            for msg_type, topic, qos in topics:
                self.pubsub.create_publisher(msg_type, topic, qos)

    def _start_control_loop(self):
        """Start the main control timer"""
        timer_period = 1.0 / FREQ
        self.vel_timer = self.create_timer(timer_period, self._timer_callback)

    # Callback methods
    def _mode_callback(self, msg: String):
        """Handle mode change commands"""
        self.mode = msg.data
        self.cluster_enable = (self.mode == "NAV_M" or self.mode == "ADPTV_NAV_M")
        if self.mode in ["JOY_M", "NEU_M"]:
            self.cluster_enable = False

    def _hw_sim_callback(self, msg: Bool):
        """Switch between hardware and simulation output"""
        previous_output = self.output
        self.output = "actual" if msg.data else "sim"
        
        if previous_output != self.output:
            self.get_logger().info(f"Changed output to {self.output}")

    def _enable_callback(self, msg: Bool):
        """Handle enable/disable commands"""
        self.enable = msg.data

    def _robot_pose_callback(self, msg: Pose2D, robot_id: str, output: str):
        """Process robot pose updates"""
        try:
            new_pose = [float(msg.x), float(msg.y), float(msg.theta)]
            
            # Get appropriate status dictionary
            status_dict = (self.actual_robot_status if output == "actual" 
                        else self.sim_robot_status)
            robot_status = status_dict[robot_id]
            
            # Calculate velocity if previous pose exists
            if robot_status.pose is None:
                new_velocity = [0.0, 0.0, 0.0]
            else:
                prev_pose = robot_status.pose
                new_velocity = [
                    float((msg.x - prev_pose[0]) * FREQ),
                    float((msg.y - prev_pose[1]) * FREQ),
                    float((msg.theta - prev_pose[2]) * FREQ)
                ]
            
            # Update robot status
            robot_status.update_pose(new_pose, new_velocity, time.time())
            
            self.get_logger().debug(
                f"Robot {output}/{robot_id} pose: {new_pose}, vel: {new_velocity}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error in _robot_pose_callback for {robot_id}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _cmd_callback(self, msg: Twist):
        """Process joystick commands for cluster navigation"""
        # Calculate command frequency
        current_time = time.time()
        freq = (1.0 / (current_time - self.joy_timestamp) 
                if self.joy_timestamp is not None else JOY_FREQ)
        self.joy_timestamp = current_time

        if self.mode == "NAV_M" or self.mode == "ADPTV_NAV_M": #only update cluster if in navigation mode
            try:
                if self.cluster.control_mode == ControlMode.POSITION:
                        # Update desired cluster position based on joystick input
                        v_x = msg.linear.x
                        v_y = msg.linear.y
                        v_r = msg.angular.z * 0.3
                        
                        self.c_des[0, 0] += v_x / freq
                        self.c_des[1, 0] += v_y / freq
                        self.c_des[2, 0] += v_r / freq
                        self.c_des[2, 0] = self._wrap_to_pi(self.c_des[2, 0])
                        
                        #self.get_logger().info(f"Cluster desired position: {self.c_des.flatten()}")
                elif self.cluster.control_mode == ControlMode.VELOCITY:
                    x = msg.linear.x
                    y = msg.linear.y
                    t = msg.angular.z
                    if self.output == "actual":
                        ct = math.cos(self.c[2])
                        st = math.sin(self.c[2])
                        self.cdot_des[0, 0] = x*ct - y*st
                        self.cdot_des[1, 0] = x*st + y*ct
                        self.cdot_des[2, 0] = self._wrap_to_pi(msg.angular.z * 0.3)
                    elif self.output == "sim":
                        ct = math.cos(self.sim_c[2])
                        st = math.sin(self.sim_c[2])
                        self.sim_cdot_des[0, 0] = x*ct - y*st
                        self.sim_cdot_des[1, 0] = x*st + y*ct
                        self.sim_cdot_des[2, 0] = self._wrap_to_pi(msg.angular.z * 0.3)
                        self.get_logger().info(f"msgs: {x}, {y}, {t} / cur_t :{self.sim_c[2]}")
                        self.get_logger().info(f"cdot_des: {self.sim_cdot_des[0, 0]}, {self.sim_cdot_des[1, 0]}, {self.sim_cdot_des[2, 0]}")
            except Exception as e:
                self.get_logger().error(f"Error in _cmd_callback: {e}")

    def _cluster_params_callback(self, msg: Float32MultiArray):
        """Update cluster formation parameters"""
        self.get_logger().info(f"Received cluster parameters: {msg.data}")
        
        # Check for reset command (all negative values)
        if all(param < 0.0 for param in msg.data[:3]):
            self.get_logger().info("Resetting cluster parameters")
            self._reset_cluster_desired_position()
            return
        
        # Update cluster parameters if configured
        if self.actual_configured or self.sim_configured:
            param_start = self.cluster_size * ROVER_DOF - len(self.cluster_params)
            param_end = self.cluster_size * ROVER_DOF
            self.c_des[param_start:param_end] = np.reshape(
                msg.data, (len(msg.data), 1)
            )

    def _cluster_desired_callback(self, msg: Float32MultiArray):
        """Update desired cluster position"""
        #self.get_logger().info(f"Received cluster desired position: {msg.data}")
        
        if self.actual_configured or self.sim_configured:
            data_length = len(msg.data)
            self.c_des[0:data_length] = np.reshape(msg.data, (data_length, 1))

    def _timer_callback(self):
        """Main control loop timer callback"""
        # Configure clusters if enough robots are available
        self._try_configure_clusters()
        
        # Publish velocity commands if cluster is enabled
        if self.cluster_enable:
            if self.output == "actual" and self.actual_configured:
                self._publish_velocities("actual")
            if self.sim_configured:
                self._publish_velocities("sim")
        else:
            if self.sim_configured:
                self._publish_desired_poses(self.sim_registered_robots)

    # Cluster management methods
    def _try_configure_clusters(self):
        """Try to configure clusters if enough robots are available"""
        if (self.output == "actual" and not self.actual_configured and 
            self._count_alive_robots("actual") >= self.cluster_size):
            self._assign_robots("actual")
        
        if (self.output == "sim" and not self.sim_configured and 
            self._count_alive_robots("sim") >= self.cluster_size):
            self._assign_robots("sim")
        #else: 
        #   self.get_logger().info(f"Not enough robots to assign sim. Output {self.output} Sim Configured {self.sim_configured} Robots Alive {self._count_alive_robots("sim")} ")

    def _assign_robots(self, output: str):
        """Assign available robots to cluster formation"""
        if output == "actual":
            status_dict = self.actual_robot_status
            self.registered_robots = []
            
            # Find available robots
            for robot_id, robot_status in status_dict.items():
                if robot_status.is_alive():
                    self.registered_robots.append(robot_id)
                if len(self.registered_robots) == self.cluster_size:
                    break
            
            self.actual_configured = True
            self.sim_registered_robots = self.registered_robots.copy()
            self.sim_configured = True
            
            self.get_logger().info(
                f"Formed actual cluster with robots: {self.registered_robots} "
                f"from list: {self.robot_id_list}"
            )
            
        elif output == "sim":
            status_dict = self.sim_robot_status
            self.sim_registered_robots = []
            
            # Find available simulation robots
            for robot_id, robot_status in status_dict.items():
                if robot_status.is_alive():
                    self.sim_registered_robots.append(robot_id)
                if len(self.sim_registered_robots) == self.cluster_size:
                    break
            
            self.sim_configured = True
            
            self.get_logger().info(
                f"Formed sim cluster with robots: {self.sim_registered_robots}"
            )

    def _count_alive_robots(self, output: str) -> int:
        """Count the number of robots reporting data"""
        status_dict = (self.actual_robot_status if output == "actual" 
                      else self.sim_robot_status)
        return sum(1 for status in status_dict.values() if status.is_alive())

    def _publish_velocities(self, output: str):
        """Compute and publish velocity commands to robots"""
        # Setup output-specific variables
        msg_prefix = '' if output == 'actual' else '/sim'
        cluster_robots = (self.registered_robots if output == "actual" 
                         else self.sim_registered_robots)
        robot_status_dict = (self.actual_robot_status if output == "actual" 
                           else self.sim_robot_status)
        
        # Handle disabled state
        if not self.enable:
            self._publish_zero_velocities(msg_prefix, cluster_robots)
            return
        
        # Gather current robot positions and velocities
        robot_positions, robot_velocities = self._gather_robot_data(
            cluster_robots, robot_status_dict
        )
        
        # Get velocity commands from cluster controller
        cdot_des = self.cdot_des if output == "actual" else self.sim_cdot_des # Desired cluster velocity
        cdot_cmd, rdot_des, c_cur = self.cluster.get_velocity_command(
            robot_positions, robot_velocities, self.c_des, cdot_des
        )

        self.sim_c = c_cur
        
        # Convert to robot-specific commands and publish
        self._compute_and_publish_robot_commands(
            msg_prefix, cluster_robots, robot_positions, rdot_des
        )
        
        # Publish cluster information
        self._publish_cluster_info(msg_prefix, c_cur, robot_positions, rdot_des)
        
        # Publish desired robot poses
        self._publish_desired_poses(cluster_robots)

    def _publish_zero_velocities(self, msg_prefix: str, cluster_robots: List[str]):
        """Publish velocity of 0 to all robots"""
        zero_vel = Twist()
        zero_vel.linear.x = 0.0
        zero_vel.angular.z = 0.0
        
        for robot_id in cluster_robots:
            self.pubsub.publish(f"{msg_prefix}/{robot_id}/cmd_vel", zero_vel)

    def _gather_robot_data(self, cluster_robots: List[str], 
                        robot_status_dict: Dict[str, RobotStatus]) -> Tuple[np.ndarray, np.ndarray]:
        """Gather robot positions and velocities into numpy arrays"""
        positions = []
        velocities = []
        
        for robot_id in cluster_robots:
            robot_status = robot_status_dict[robot_id]
            
            # Add None check
            if robot_status.pose is None or robot_status.velocity is None:
                self.get_logger().warn(f"Robot {robot_id} has no pose or velocity data")
                positions.extend([0.0, 0.0, 0.0])
                velocities.extend([0.0, 0.0, 0.0])
            else:
                pose = [float(x) for x in robot_status.pose]
                vel = [float(x) for x in robot_status.velocity]
                positions.extend(pose)
                velocities.extend(vel)
        
        try:
            robot_positions = np.array(positions, dtype=np.float64).reshape((self.cluster_size * ROVER_DOF, 1))
            robot_velocities = np.array(velocities, dtype=np.float64).reshape((self.cluster_size * ROVER_DOF, 1))
            
            #self.get_logger().info(f"Robot positions shape: {robot_positions.shape}")
            #self.get_logger().info(f"Robot velocities shape: {robot_velocities.shape}")
            
            return robot_positions, robot_velocities
            
        except Exception as e:
            self.get_logger().error(f"Error in _gather_robot_data: {e}")
            zero_positions = np.zeros((self.cluster_size * ROVER_DOF, 1), dtype=np.float64)
            zero_velocities = np.zeros((self.cluster_size * ROVER_DOF, 1), dtype=np.float64)
            return zero_positions, zero_velocities

    def _compute_and_publish_robot_commands(self, msg_prefix: str, cluster_robots: List[str], 
                                          robot_positions: np.ndarray, rdot_des: np.ndarray):
        """Compute individual robot commands and publish them"""
        rover_commands = []
        
        # Due to non-holonomic constraints we need to convert x,y velocities to linear/angular velocities
        for i in range(len(cluster_robots)):
            x_vel = float(rdot_des[i * ROVER_DOF + 0, 0])
            y_vel = float(rdot_des[i * ROVER_DOF + 1, 0])
            current_theta = robot_positions[i * ROVER_DOF + 2, 0]
            
            self.get_logger().info(f"p{i+1}")
            linear_cmd, angular_cmd = self._compute_robot_command(
                x_vel, y_vel, current_theta
            )
            rover_commands.append([linear_cmd, angular_cmd])
        
        # Limit velocities based on physical constraints
        rover_commands = self._apply_velocity_limits(rover_commands)
        
        # Publish robot velocity commands
        for i, robot_id in enumerate(cluster_robots):
            vel_msg = Twist()
            vel_msg.linear.x = rover_commands[i][0]
            vel_msg.angular.z = rover_commands[i][1]
            
            try:
                topic = f"{msg_prefix}/{robot_id}/cmd_vel"
                self.pubsub.publish(topic, vel_msg)
                self.get_logger().info(
                    f"Robot {i} velocity[{topic}]: "
                    f"linear={vel_msg.linear.x:.3f}, angular={vel_msg.angular.z:.3f}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to publish velocity command: {e}")

    def _compute_robot_command(self, x_vel: float, y_vel: float, 
                              current_theta: float) -> Tuple[float, float]:
        """Heading controller to compute linear and angular velocity commands for a single robot"""
        try:
            translation_magnitude = math.sqrt(x_vel**2 + y_vel**2)
            
            # If in position control mode check if robot is close enough to desired position
            if self.control_mode == ControlMode.POSITION and translation_magnitude < EPSILON * KP_GAIN: #multiply by gain to convert to m/s
                return 0.0, 0.0
            
            # Compute desired heading and angular error
            desired_angle = -math.atan2(x_vel, y_vel) #x and y are swapped to get angle from y axis
            angular_error = desired_angle - current_theta #-2pi to 2pi
            #Get angular error for shortest path to desired angle
            angular_error = angular_error%(2*np.pi) 
            if(angular_error > np.pi):
                angular_error = angular_error - 2*np.pi
            
            self.get_logger().info(
                f"x_vel={x_vel:.3f}, y_vel={y_vel:.3f}, "
                f"current_theta={current_theta:.3f}, desired_angle={desired_angle:.3f}"
            )
            
            # If angle error is small, move forward otherwise move backward to avoid large turns
            if(abs(angular_error) < math.pi / 2):
                linear_vel = translation_magnitude * math.cos(abs(angular_error))
                angular_vel = angular_error
                #self.get_logger().info(f"Angular error small going forward {angular_error} with velocities {linear_vel}, {angular_vel}")
            else:
                # Move backward if angle error is large
                #self.get_logger().info(f"Angular error too large {angular_error}")
                corrected_error = self._wrap_to_pi(math.pi - angular_error)
                linear_vel = -translation_magnitude * math.cos(abs(corrected_error))
                angular_vel = -corrected_error
            self.get_logger().info(
                f"v_t={linear_vel:.3f}, v_r={angular_vel:.3f}"
            )
            
            return linear_vel, angular_vel
        except Exception as e:
            self.get_logger().error(f"Error in _publish_desired_poses: {e}")

    def _apply_velocity_limits(self, rover_commands: List[List[float]]) -> np.ndarray:
        """Apply velocity limits to robot commands"""
        commands = np.array(rover_commands)
        
        for i in range(len(commands)):
            # Limit angular velocity
            commands[i][1] = np.clip(commands[i][1], -MAX_VEL, MAX_VEL)
            
            # Limit linear velocity considering angular velocity
            remaining_vel = MAX_VEL - abs(commands[i][1])
            commands[i][0] = np.clip(commands[i][0], -remaining_vel, remaining_vel)
        
        return commands

    def _publish_cluster_info(self, msg_prefix: str, cluster_current: np.ndarray, 
                             robot_positions: np.ndarray, robot_desired_vel: np.ndarray):
        """Publish cluster status information"""
        cluster_msg = ClusterInfo()
        cluster_msg.cluster.data = cluster_current.flatten().tolist()
        cluster_msg.cluster_desired.data = self.c_des.flatten().tolist()
        cluster_msg.rover.data = robot_positions.flatten().tolist()
        cluster_msg.rover_desired.data = robot_desired_vel.flatten().tolist()
        
        self.pubsub.publish(f"{msg_prefix}/cluster_info", cluster_msg)

    def _publish_desired_poses(self, cluster_robots: List[str]):
        """Publish desired poses for each robot"""
        try:
            desired_positions = self.cluster.get_desired_robot_positions(self.c_des)
            
            if desired_positions is None:
                self.get_logger().warn("get_desired_position returned None")
                return
                
            #self.get_logger().info(f"Desired positions shape: {desired_positions.shape}")
            #self.get_logger().info(f"Desired positions: {desired_positions.flatten()}")
            
            for i, robot_id in enumerate(cluster_robots):
                pose_msg = Pose2D()
                start_idx = i * ROVER_DOF

                if hasattr(desired_positions, 'shape') and len(desired_positions.shape) == 2:
                    pose_msg.x = float(desired_positions[start_idx, 0])
                    pose_msg.y = float(desired_positions[start_idx + 1, 0])
                    pose_msg.theta = float(desired_positions[start_idx + 2, 0])
                else:
                    pose_msg.x = float(desired_positions[start_idx])
                    pose_msg.y = float(desired_positions[start_idx + 1])
                    pose_msg.theta = float(desired_positions[start_idx + 2])
                
                #self.get_logger().info(
                #    f"Publishing desired pose for {robot_id}: "
                #    f"x={pose_msg.x:.3f}, y={pose_msg.y:.3f}, theta={pose_msg.theta:.3f}"
                #)
                
                self.pubsub.publish(f"/{robot_id}/desiredPose2D", pose_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in _publish_desired_poses: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _reset_cluster_desired_position(self):
        """Reset cluster desired position to initial parameters"""
        self.c_des = np.zeros((self.cluster_size * ROVER_DOF, 1))
        param_start = self.cluster_size * ROVER_DOF - len(self.cluster_params)
        param_end = self.cluster_size * ROVER_DOF
        self.c_des[param_start:param_end] = np.reshape(
            self.cluster_params, (len(self.cluster_params), 1)
        )

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap angle to [-pi, pi] range"""
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    """Main entry point for the controller node"""
    rclpy.init(args=args)
    
    try:
        controller = Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in controller: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()