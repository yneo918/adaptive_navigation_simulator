import math

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.node import Node


class GoalPoseForwarder(Node):
    def __init__(self) -> None:
        super().__init__('goal_pose_forwarder')
        self.pose_publisher = self.create_publisher(Pose2D, '/rviz/goal_pose2D', 10)
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._on_goal_pose,
            10,
        )

    def _on_goal_pose(self, message: PoseStamped) -> None:
        self.get_logger().info('Received goal pose')
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        ) - math.pi / 2.0  # Adjust for Pioneer orientation

        pose_2d = Pose2D()
        pose_2d.x = message.pose.position.x
        pose_2d.y = message.pose.position.y
        pose_2d.theta = yaw
        self.pose_publisher.publish(pose_2d)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None
    try:
        node = GoalPoseForwarder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
