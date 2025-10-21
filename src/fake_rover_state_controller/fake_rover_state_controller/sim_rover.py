import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist

from .my_ros_module import PubSubManager

UPDATE_RATE_HZ = 10.0
MASS_KG = 10.0
INERTIA_KGM2 = 0.75
RESPONSE_TIME_S = 0.6
ANG_RESPONSE_TIME_S = 0.5
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0
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
            ],
        )

        self.robot_id = self.get_parameter('robot_id').value
        self.prefix = self.get_parameter('prefix').value

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
        self.pubsub.create_publisher(
            Pose2D,
            f'{self.prefix}/{self.robot_id}/pose2D',
            10,
        )

        timer_period = 1.0 / UPDATE_RATE_HZ
        self.timer = self.create_timer(timer_period, self._on_timer)

    def _command_callback(self, msg: Twist) -> None:
        self.target_velocity['linear'] = _clamp(msg.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        self.target_velocity['angular'] = _clamp(msg.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        self.last_command_time = self.get_clock().now()

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
        linear_error = self.target_velocity['linear'] - self.current_velocity['linear']
        force_command = MASS_KG * linear_error / RESPONSE_TIME_S
        drag_force = -LINEAR_DRAG_COEFF * self.current_velocity['linear']
        net_force = force_command + drag_force
        linear_accel = _clamp(net_force / MASS_KG, -MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL)
        self.current_velocity['linear'] += linear_accel * dt
        self.current_velocity['linear'] = _clamp(self.current_velocity['linear'], -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)

        angular_error = self.target_velocity['angular'] - self.current_velocity['angular']
        torque_command = INERTIA_KGM2 * angular_error / ANG_RESPONSE_TIME_S
        drag_torque = -ANGULAR_DRAG_COEFF * self.current_velocity['angular']
        net_torque = torque_command + drag_torque
        angular_accel = _clamp(net_torque / INERTIA_KGM2, -MAX_ANG_ACCEL, MAX_ANG_ACCEL)
        self.current_velocity['angular'] += angular_accel * dt
        self.current_velocity['angular'] = _clamp(self.current_velocity['angular'], -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        heading_mid = self.position['theta'] + 0.5 * self.current_velocity['angular'] * dt
        self.position['theta'] = self._wrap_to_pi(self.position['theta'] + self.current_velocity['angular'] * dt)
        self.position['x'] += self.current_velocity['linear'] * (-math.sin(heading_mid)) * dt
        self.position['y'] += self.current_velocity['linear'] * math.cos(heading_mid) * dt

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
