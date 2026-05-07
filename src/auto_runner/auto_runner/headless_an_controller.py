"""Headless replacement for gui_adaptive_navigation.

Performs the same ROS-side responsibilities as the PyQt GUI without any GUI:
  - Subscribes /rviz/pose2D and forwards via inverse kinematics to /p{i}/set_pose2D
  - Publishes /ctrl/{enable, cluster_mode, adaptive_mode, cluster_params} at 10Hz
  - Accepts external configuration via /auto/config (JSON String)
  - Accepts /auto/start, /auto/stop (Bool) to enable/disable navigation

Config JSON example:
  {"d": 100.0, "adaptive_mode": "MAX", "cluster_mode": "ADPTV_NAV_M"}

Cluster mode "ADPTV_NAV_M" enables adaptive navigation (matches existing
adaptive_nav.py contract).
"""

import json
import math
import sys

import numpy as np
import sympy as sp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, Int16, String, Float32MultiArray, Float64

from gui_package.my_ros_module import PubSubManager
from adaptive_nav.ScalarGradient import ControlMode
from cluster_node.Cluster import PentagonLeaderConfig


DEFAULT_QOS = 10
LOW_QOS = 1
N_ROVER = 5


class HeadlessANController(Node):
    """Headless adaptive navigation controller (no GUI)."""

    def __init__(self):
        super().__init__('headless_an_controller')

        # Cluster geometry parameters (defaults match GUI initial values)
        self.cluster_d = 10.0
        self.cluster_b3 = 0.0
        self.cluster_b4 = 0.0
        self.cluster_b5 = 0.0

        # Control state
        self.enable = False
        self.cluster_mode = 'NEUTRAL'
        self.adaptive_mode = ControlMode.MAX.value
        self.select = 1
        self.z_des = 0.6  # CROSSTRACK target contour value

        self.pubsub = PubSubManager(self)
        self._setup_publishers()
        self._setup_subscriptions()
        self._init_inverse_kinematics()

        self.timer = self.create_timer(0.1, self._timer_callback)  # 10Hz
        self.get_logger().info('HeadlessANController ready')

    def _setup_publishers(self):
        self.pubsub.create_publisher(Bool, '/ctrl/enable', DEFAULT_QOS)
        self.pubsub.create_publisher(Int16, '/ctrl/select_rover', LOW_QOS)
        self.pubsub.create_publisher(String, '/ctrl/cluster_mode', LOW_QOS)
        self.pubsub.create_publisher(String, '/ctrl/adaptive_mode', LOW_QOS)
        self.pubsub.create_publisher(
            Float32MultiArray, '/ctrl/cluster_params', DEFAULT_QOS)
        self.pubsub.create_publisher(Float64, '/ctrl/z_des', DEFAULT_QOS)
        for i in range(1, N_ROVER + 1):
            self.pubsub.create_publisher(Pose2D, f'/p{i}/set_pose2D', 10)

    def _setup_subscriptions(self):
        self.pubsub.create_subscription(
            Pose2D, '/rviz/pose2D', self._rviz_pose_callback, 10)
        self.pubsub.create_subscription(
            String, '/auto/config', self._config_callback, 10)
        self.pubsub.create_subscription(
            Bool, '/auto/start', self._start_callback, 10)
        self.pubsub.create_subscription(
            Bool, '/auto/stop', self._stop_callback, 10)

    def _init_inverse_kinematics(self):
        config = PentagonLeaderConfig()
        _, ikine, _, _, _ = config.setup_kinematics()
        c_symbols = sp.symbols('c0:15')
        self._ikine_func = sp.lambdify(c_symbols, ikine, 'numpy')

    def _publish_cluster_params(self):
        msg = Float32MultiArray()
        msg.data = [
            self.cluster_d, self.cluster_d,
            self.cluster_d, self.cluster_d,
            self.cluster_b3, self.cluster_b4, self.cluster_b5
        ]
        self.pubsub.publish('/ctrl/cluster_params', msg)

    def _timer_callback(self):
        # Selected rover (always p1 for batch)
        self.pubsub.publish('/ctrl/select_rover', Int16(data=self.select))
        # Cluster mode (NEUTRAL or ADPTV_NAV_M)
        self.pubsub.publish('/ctrl/cluster_mode', String(data=self.cluster_mode))
        # Adaptive mode (MAX, MIN, RIDGE_DOWN, ...)
        self.pubsub.publish('/ctrl/adaptive_mode', String(data=self.adaptive_mode))
        # Enable
        self.pubsub.publish('/ctrl/enable', Bool(data=self.enable))
        # Cluster params (publish continuously so latecomers get them)
        self._publish_cluster_params()
        # z_des (publish continuously so adaptive_nav stays in sync)
        self.pubsub.publish('/ctrl/z_des', Float64(data=self.z_des))

    def _config_callback(self, msg: String):
        """Update configuration from JSON string."""
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad /auto/config JSON: {e}')
            return

        if 'd' in cfg:
            self.cluster_d = float(cfg['d'])
            self.get_logger().info(f'  d set to {self.cluster_d}')
        if 'b3' in cfg:
            self.cluster_b3 = float(cfg['b3'])
        if 'b4' in cfg:
            self.cluster_b4 = float(cfg['b4'])
        if 'b5' in cfg:
            self.cluster_b5 = float(cfg['b5'])
        if 'adaptive_mode' in cfg:
            mode = cfg['adaptive_mode']
            if any(m.value == mode for m in ControlMode):
                self.adaptive_mode = mode
                self.get_logger().info(f'  adaptive_mode set to {mode}')
            else:
                self.get_logger().error(f'Unknown adaptive_mode: {mode}')
        if 'cluster_mode' in cfg:
            self.cluster_mode = cfg['cluster_mode']
            self.get_logger().info(f'  cluster_mode set to {self.cluster_mode}')
        if 'z_des' in cfg:
            self.z_des = float(cfg['z_des'])
            self.get_logger().info(f'  z_des set to {self.z_des}')
            self.pubsub.publish('/ctrl/z_des', Float64(data=self.z_des))

        # Publish params immediately so the cluster controller picks up the new d.
        self._publish_cluster_params()

    def _start_callback(self, msg: Bool):
        self.enable = bool(msg.data)
        if self.enable:
            self.cluster_mode = 'ADPTV_NAV_M'
        self.get_logger().info(
            f'/auto/start={self.enable} cluster_mode={self.cluster_mode}')

    def _stop_callback(self, msg: Bool):
        if bool(msg.data):
            self.enable = False
            self.cluster_mode = 'NEUTRAL'
            self.get_logger().info('/auto/stop received: navigation halted')

    def _rviz_pose_callback(self, msg: Pose2D):
        """Convert cluster pose to per-robot poses via inverse kinematics."""
        x_c, y_c, theta_c = msg.x, msg.y, msg.theta

        # 15-element cluster state: [x_c, y_c, theta_c, phi1-5, d2-5, beta3-5]
        c_state = [
            x_c, y_c, theta_c,
            0.0, 0.0, 0.0, 0.0, 0.0,                     # phi1-5
            self.cluster_d, self.cluster_d,
            self.cluster_d, self.cluster_d,              # d2-5
            self.cluster_b3, self.cluster_b4, self.cluster_b5
        ]

        robot_positions = np.array(self._ikine_func(*c_state)).flatten()
        self.get_logger().info(
            f'rviz/pose2D received: cluster=({x_c:.1f},{y_c:.1f},{theta_c:.2f}) '
            f'd={self.cluster_d}')

        for i in range(N_ROVER):
            pose_msg = Pose2D()
            pose_msg.x = float(robot_positions[i * 3])
            pose_msg.y = float(robot_positions[i * 3 + 1])
            pose_msg.theta = float(robot_positions[i * 3 + 2])
            self.pubsub.publish(f'/p{i + 1}/set_pose2D', pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadlessANController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
