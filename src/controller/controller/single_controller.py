import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose2D, Twist

from .my_ros_module import PubSubManager
from enum import Enum

import time
import math


UPDATE_RATE_HZ = 10.0
OFFSET_ANGLE_TOLERANCE = 0.05  # radians

class ControlMode(Enum):
    IMU_CALIBRATION = 1
    GO_NORTH = 2

    def __str__(self) -> str:
        return self.name.lower()


class SingleController(Node):
    def __init__(self) -> None:
        super().__init__('single_controller')
        self.pubsub = PubSubManager(self)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 'p0'),
            ],
        )

        self.robot_id = self.get_parameter('robot_id').value

        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.velocity = {'linear': 0.0, 'angular': 0.0}

        self.pubsub.create_publisher(
            Twist,
            f'/{self.robot_id}/cmd_vel',
            10,
        )
        self.pubsub.create_publisher(
            Float32,
            f'/{self.robot_id}/imu_offset',
            10,
        )

        self.pubsub.create_subscription(
            Pose2D,
            f'/{self.robot_id}/pose2D',
            self.pose_callback,
            10,
        )

        self.pubsub.create_subscription(
            String,
            f'/{self.robot_id}/control_mode',
            self.control_mode_callback,
            10, 
        )

    def pose_callback(self, msg: Pose2D) -> None:
        self.position['x'] = msg.x
        self.position['y'] = msg.y
        self.position['theta'] = msg.theta
    
    def control_mode_callback(self, msg: String) -> None:
        mode_str = msg.data.lower()
        try:
            mode = ControlMode[mode_str.upper()]
            self.get_logger().info(f'Switched to control mode: {mode}')
            if mode == ControlMode.IMU_CALIBRATION:
                self.start_imu_calibration()
            elif mode == ControlMode.GO_NORTH:
                self.start_go_north()
        except KeyError:
            self.get_logger().error(f'Unknown control mode: {mode_str}')

    def start_imu_calibration(self) -> None:
        self.get_logger().info('Starting IMU calibration...')
        angle = self.start_go_north()
        while not (-OFFSET_ANGLE_TOLERANCE < angle < OFFSET_ANGLE_TOLERANCE):
            self.get_logger().info('Re-calibrating IMU...')
            angle = self.start_go_north()
        self.get_logger().info('IMU calibration completed successfully!')

    
    def start_go_north(self, offset = 0.0, moving_time = 5) -> float:
        self.get_logger().info('Starting to go north...')
        vel_msg = Twist()
        while not (-OFFSET_ANGLE_TOLERANCE < self.position['theta'] - offset < OFFSET_ANGLE_TOLERANCE):
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.pubsub.publish(f'/{self.robot_id}/cmd_vel', vel_msg)
            time.sleep(1.0)
            while not (-OFFSET_ANGLE_TOLERANCE < self.position['theta'] - offset < OFFSET_ANGLE_TOLERANCE):
                vel_msg = Twist()
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.1
                self.pubsub.publish(f'/{self.robot_id}/cmd_vel', vel_msg)
                time.sleep(1.0 / UPDATE_RATE_HZ)
        start_pose = self.position.copy()
        for _ in range(int(UPDATE_RATE_HZ * moving_time)):  # Move north
            vel_msg = Twist()
            vel_msg.linear.x = 1.0
            vel_msg.angular.z = 0.0
            self.pubsub.publish(f'/{self.robot_id}/cmd_vel', vel_msg)
            time.sleep(1.0 / UPDATE_RATE_HZ)
        end_pose = self.position.copy()
        self.get_logger().info(f'Go north completed. Start pose: {start_pose}, End pose: {end_pose}')
        diff_x = end_pose['x'] - start_pose['x']
        diff_y = end_pose['y'] - start_pose['y']
        self.get_logger().info(f'Pose difference while going north: Δx={diff_x}, Δy={diff_y}')
        angle = math.atan2(diff_y, diff_x)
        if -OFFSET_ANGLE_TOLERANCE < angle < OFFSET_ANGLE_TOLERANCE:
            self.get_logger().info('Successfully moved north!')
        else:
            self.get_logger().info('Failed to move north accurately.')
        return angle