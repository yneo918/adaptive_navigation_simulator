import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist

from .my_ros_module import PubSubManager

UPDATE_RATE_HZ = 10.0
MAX_UPDATE_RATE_HZ = 10000.0  # Maximum update rate for time scaling (supports time_scale up to 1000)
TARGET_SUBSTEP_DT = 0.1  # Target substep size for integration (seconds)
MIN_TIME_SCALE = 0.01
MAX_TIME_SCALE = 1000.0
MASS_KG = 10.0
INERTIA_KGM2 = 0.75
RESPONSE_TIME_S = 0.6
ANG_RESPONSE_TIME_S = 0.5
MAX_LINEAR_SPEED = 10.0
MAX_ANGULAR_SPEED = 10.0
MAX_FORCE_N = 45.0
MAX_TORQUE_NM = 6.0
LINEAR_DRAG_COEFF = 8.0
ANGULAR_DRAG_COEFF = 1.5
COMMAND_TIMEOUT_S = 0.5

MAX_LINEAR_ACCEL = MAX_FORCE_N / MASS_KG
MAX_ANG_ACCEL = MAX_TORQUE_NM / INERTIA_KGM2
DEFAULT_PREFIX = '/sim'


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


class SimRover(Node):
    def __init__(self) -> None:
        super().__init__('sim_rover')
        self.pubsub = PubSubManager(self)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 'p0'),
                ('x', 0.0),
                ('y', 0.0),
                ('t', 0.0),
                ('prefix', DEFAULT_PREFIX),
                ('time_scale', 1.0),
            ],
        )

        self.robot_id = self.get_parameter('robot_id').value
        self.prefix = self.get_parameter('prefix').value
        self.time_scale = float(self.get_parameter('time_scale').value)

        # Validate time_scale range
        if not (MIN_TIME_SCALE <= self.time_scale <= MAX_TIME_SCALE):
            self.get_logger().error(
                f'Robot {self.robot_id}: Invalid time_scale={self.time_scale}. '
                f'Must be between {MIN_TIME_SCALE} and {MAX_TIME_SCALE}'
            )
            raise ValueError(f'time_scale out of range: {self.time_scale}')

        # Warn if time_scale is very high
        if self.time_scale > 100.0:
            effective_rate = min(UPDATE_RATE_HZ * self.time_scale, MAX_UPDATE_RATE_HZ)
            timer_period = 1.0 / effective_rate
            estimated_dt = timer_period * self.time_scale
            num_substeps = max(1, int(estimated_dt / TARGET_SUBSTEP_DT + 0.5))

            self.get_logger().warn(
                f'⚠ Robot {self.robot_id}: time_scale={self.time_scale} is very high. '
                f'Update rate: {effective_rate:.0f} Hz, Substeps: {num_substeps}'
            )

        self.position = {
            'x': float(self.get_parameter('x').value),
            'y': float(self.get_parameter('y').value),
            'theta': float(self.get_parameter('t').value),
        }
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        self.target_velocity = {'linear': 0.0, 'angular': 0.0}
        self.last_command_time = self.get_clock().now()
        self.last_update_time = None

        self.pubsub.create_subscription(
            Twist,
            f'{self.prefix}/{self.robot_id}/cmd_vel',
            self._command_callback,
            10,
        )
        self.pubsub.create_subscription(
            Pose2D,
            f'{self.prefix}/{self.robot_id}/set_pose2D',
            self._set_pose_callback,
            10,
        )
        self.pubsub.create_publisher(
            Pose2D,
            f'{self.prefix}/{self.robot_id}/pose2D',
            10,
        )

        # Adjust timer frequency based on time_scale for smooth simulation
        # Cap at MAX_UPDATE_RATE_HZ to prevent unrealistic frequencies
        effective_rate = min(UPDATE_RATE_HZ * self.time_scale, MAX_UPDATE_RATE_HZ)
        timer_period = 1.0 / effective_rate
        self.timer = self.create_timer(timer_period, self._on_timer)

    def _command_callback(self, msg: Twist) -> None:
        self.target_velocity['linear'] = _clamp(msg.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        self.target_velocity['angular'] = _clamp(msg.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        self.last_command_time = self.get_clock().now()

    def _set_pose_callback(self, msg: Pose2D) -> None:
        self.position['x'] = msg.x
        self.position['y'] = msg.y
        self.position['theta'] = self._wrap_to_pi(msg.theta)
        self.current_velocity['linear'] = 0.0
        self.current_velocity['angular'] = 0.0
        self.target_velocity['linear'] = 0.0
        self.target_velocity['angular'] = 0.0
        self.last_command_time = self.get_clock().now()
        self._publish_pose()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self.last_update_time is None:
            dt = 1.0 / UPDATE_RATE_HZ
        else:
            dt = (now - self.last_update_time).nanoseconds / 1e9
            if dt <= 0.0:
                dt = 1.0 / UPDATE_RATE_HZ
        self.last_update_time = now

        if (now - self.last_command_time).nanoseconds / 1e9 > COMMAND_TIMEOUT_S:
            self.target_velocity['linear'] = 0.0
            self.target_velocity['angular'] = 0.0

        self._integrate_dynamics(dt)
        self._publish_pose()

    def _integrate_dynamics(self, dt: float) -> None:
        """
        Integrate robot dynamics with automatic substep subdivision.

        For large dt (e.g., high time_scale), subdivides into smaller steps
        to maintain numerical accuracy and preserve physical response characteristics.
        """
        # Calculate number of substeps needed
        num_substeps = max(1, int(dt / TARGET_SUBSTEP_DT + 0.5))
        sub_dt = dt / num_substeps

        # Integrate over substeps
        for _ in range(num_substeps):
            # Linear dynamics
            linear_error = self.target_velocity['linear'] - self.current_velocity['linear']
            force_command = MASS_KG * linear_error / RESPONSE_TIME_S
            drag_force = -LINEAR_DRAG_COEFF * self.current_velocity['linear']
            net_force = force_command + drag_force
            linear_accel = _clamp(net_force / MASS_KG, -MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL)
            self.current_velocity['linear'] += linear_accel * sub_dt
            self.current_velocity['linear'] = _clamp(
                self.current_velocity['linear'], -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED
            )

            # Angular dynamics
            angular_error = self.target_velocity['angular'] - self.current_velocity['angular']
            torque_command = INERTIA_KGM2 * angular_error / ANG_RESPONSE_TIME_S
            drag_torque = -ANGULAR_DRAG_COEFF * self.current_velocity['angular']
            net_torque = torque_command + drag_torque
            angular_accel = _clamp(net_torque / INERTIA_KGM2, -MAX_ANG_ACCEL, MAX_ANG_ACCEL)
            self.current_velocity['angular'] += angular_accel * sub_dt
            self.current_velocity['angular'] = _clamp(
                self.current_velocity['angular'], -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED
            )

            # Position update (REP103: X+ forward, Y+ left, counter-clockwise rotation positive)
            heading_mid = self.position['theta'] + 0.5 * self.current_velocity['angular'] * sub_dt
            self.position['theta'] = self._wrap_to_pi(
                self.position['theta'] + self.current_velocity['angular'] * sub_dt
            )
            self.position['x'] += self.current_velocity['linear'] * math.cos(heading_mid) * sub_dt
            self.position['y'] += self.current_velocity['linear'] * math.sin(heading_mid) * sub_dt

    def _publish_pose(self) -> None:
        pose = Pose2D()
        pose.x = self.position['x']
        pose.y = self.position['y']
        pose.theta = self.position['theta']
        self.pubsub.publish(f'{self.prefix}/{self.robot_id}/pose2D', pose)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimRover()
        rclpy.spin(node)
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
