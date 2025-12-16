import math

import rclpy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPoseForwarder(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_forwarder')
        self.pose_publisher = self.create_publisher(Pose2D, '/rviz/pose2D', 10)
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initial_pose,
            10,
        )

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.get_logger().info('Received initial pose')
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

        pose_2d = Pose2D()
        pose_2d.x = message.pose.pose.position.x
        pose_2d.y = message.pose.pose.position.y
        pose_2d.theta = yaw
        self.pose_publisher.publish(pose_2d)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None
    try:
        node = InitialPoseForwarder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
