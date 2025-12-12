import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .my_ros_module import PubSubManager

DEFAULT_PUBLISH_RATE = 10.0


class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.pubsub = PubSubManager(self)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 'p0'),
                ('publish_rate', DEFAULT_PUBLISH_RATE),
                ('with_desired', False),
            ],
        )

        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        self.robot_id = self.get_parameter('robot_id').value
        publish_rate = self.get_parameter('publish_rate').value
        self.with_desired = self.get_parameter('with_desired').value

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static TF (base_link -> rover_link)
        self._publish_static_tf()

        # Position state
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.position_desired = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # Subscribe to pose data
        self.pubsub.create_subscription(
            Pose2D,
            f'/{self.robot_id}/pose2D',
            self._pose_callback,
            10,
        )

        if self.with_desired:
            self.pubsub.create_subscription(
                Pose2D,
                f'/{self.robot_id}/desiredPose2D',
                self._desired_pose_callback,
                10,
            )

        # Timer for publishing TF
        self.create_timer(1.0 / publish_rate, self._publish_tf)

    def _publish_static_tf(self):
        """Publish static transform: base_link -> rover_link"""
        static_transforms = []

        # Main robot
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{self.robot_id}/base_link'
        t.child_frame_id = f'{self.robot_id}/rover_link'
        t.transform.translation.z = 0.02
        t.transform.rotation.w = 1.0
        static_transforms.append(t)

        # Desired robot
        if self.with_desired:
            t_desired = TransformStamped()
            t_desired.header.stamp = self.get_clock().now().to_msg()
            t_desired.header.frame_id = f'{self.robot_id}desired/base_link'
            t_desired.child_frame_id = f'{self.robot_id}desired/rover_link'
            t_desired.transform.translation.z = 0.02
            t_desired.transform.rotation.w = 1.0
            static_transforms.append(t_desired)

        self.static_broadcaster.sendTransform(static_transforms)

    def _publish_tf(self):
        """Publish dynamic transform: world -> base_link"""
        now = self.get_clock().now().to_msg()
        transforms = []

        # Main robot
        transforms.append(
            self._create_transform(
                now, 'world', f'{self.robot_id}/base_link', self.position
            )
        )

        # Desired robot
        if self.with_desired:
            transforms.append(
                self._create_transform(
                    now, 'world', f'{self.robot_id}desired/base_link', self.position_desired
                )
            )

        self.tf_broadcaster.sendTransform(transforms)

    def _create_transform(
        self, stamp, parent: str, child: str, pos: dict
    ) -> TransformStamped:
        """Create TransformStamped from position dict"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = pos['x']
        t.transform.translation.y = pos['y']
        t.transform.translation.z = 0.0

        # Convert yaw to quaternion (rotation around Z axis)
        half_yaw = pos['theta'] / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)

        return t

    def _pose_callback(self, msg: Pose2D):
        self.position = {'x': msg.x, 'y': msg.y, 'theta': msg.theta}

    def _desired_pose_callback(self, msg: Pose2D):
        self.position_desired = {'x': msg.x, 'y': msg.y, 'theta': msg.theta}


def main(args=None):
    rclpy.init(args=args)

    node = TFBroadcaster()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
