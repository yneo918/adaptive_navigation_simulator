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
MAX_LINEAR_SPEED = 3.0
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
        self.current_velocity = {'x':0.0, 'y':0.0, 'angular': 0.0}
        self.target_velocity = {'x':0.0, 'y':0.0, 'angular': 0.0}
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

        timer_period = 1.0 / UPDATE_RATE_HZ
        self.timer = self.create_timer(timer_period, self._on_timer)

    def _command_callback(self, msg: Twist) -> None:
        self.target_velocity['x'] = _clamp(msg.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        self.target_velocity['y'] = _clamp(msg.linear.y, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        self.target_velocity['angular'] = _clamp(msg.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        self.last_command_time = self.get_clock().now()

    def _set_pose_callback(self, msg: Pose2D) -> None:
        self.position['x'] = msg.x
        self.position['y'] = msg.y
        self.position['theta'] = self._wrap_to_pi(msg.theta)
        self.current_velocity['x'] = 0.0
        self.current_velocity['y'] = 0.0
        self.current_velocity['angular'] = 0.0
        self.target_velocity['x'] = 0.0
        self.target_velocity['y'] = 0.0
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
            self.target_velocity['x'] = 0.0
            self.target_velocity['y'] = 0.0
            self.target_velocity['angular'] = 0.0

        self._integrate_dynamics(dt)
        self._publish_pose()

    def _integrate_dynamics(self, dt: float) -> None:
        # X方向の速度制御
        x_error = self.target_velocity['x'] - self.current_velocity['x']
        force_x = MASS_KG * x_error / RESPONSE_TIME_S
        drag_x = -LINEAR_DRAG_COEFF * self.current_velocity['x']
        net_force_x = force_x + drag_x
        accel_x = _clamp(net_force_x / MASS_KG, -MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL)
        self.current_velocity['x'] += accel_x * dt
        self.current_velocity['x'] = _clamp(self.current_velocity['x'], -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)

        # Y方向の速度制御
        y_error = self.target_velocity['y'] - self.current_velocity['y']
        force_y = MASS_KG * y_error / RESPONSE_TIME_S
        drag_y = -LINEAR_DRAG_COEFF * self.current_velocity['y']
        net_force_y = force_y + drag_y
        accel_y = _clamp(net_force_y / MASS_KG, -MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL)
        self.current_velocity['y'] += accel_y * dt
        self.current_velocity['y'] = _clamp(self.current_velocity['y'], -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)

        # 角速度制御
        angular_error = self.target_velocity['angular'] - self.current_velocity['angular']
        torque_command = INERTIA_KGM2 * angular_error / ANG_RESPONSE_TIME_S
        drag_torque = -ANGULAR_DRAG_COEFF * self.current_velocity['angular']
        net_torque = torque_command + drag_torque
        angular_accel = _clamp(net_torque / INERTIA_KGM2, -MAX_ANG_ACCEL, MAX_ANG_ACCEL)
        self.current_velocity['angular'] += angular_accel * dt
        self.current_velocity['angular'] = _clamp(self.current_velocity['angular'], -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        # REP103: transform body frame velocity to world frame (X+ forward, Y+ left)
        theta = self.position['theta']
        self.position['theta'] = self._wrap_to_pi(theta + self.current_velocity['angular'] * dt)

        vx_body = self.current_velocity['x']
        vy_body = self.current_velocity['y']
        self.position['x'] += (vx_body * math.cos(theta) - vy_body * math.sin(theta)) * dt
        self.position['y'] += (vx_body * math.sin(theta) + vy_body * math.cos(theta)) * dt

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
