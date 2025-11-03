# comments in English
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D, Twist
import math

class ArrowPublisher(Node):
    def __init__(self):
        super().__init__('arrow_publisher')
        self.create_subscription(Pose2D, '/ctrl/cluster_pose2d', self.pose_callback, 10)
        self.create_subscription(Twist, '/ctrl/cmd_vel', self.cmd_vel_callback, 10)
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.cmd_vel = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_arrow)

    def publish_arrow(self):
        marker = Marker()
        marker.header.frame_id = "world"   # must match RViz Fixed Frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "cluster_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # position
        marker.pose.position.x = self.pose['x']
        marker.pose.position.y = self.pose['y']
        marker.pose.position.z = 0.0

        # orientation (identity)
        yaw = math.atan2(self.cmd_vel['y'], self.cmd_vel['x']) + self.pose['theta']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        # arrow size
        length = math.hypot(self.cmd_vel['x'], self.cmd_vel['y'])
        marker.scale.x = length
        marker.scale.y = 0.06
        marker.scale.z = 0.12

        # color (with alpha!)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher_.publish(marker)
        self.get_logger().info('Publishing arrow marker')

    def pose_callback(self, msg: Pose2D):
        self.pose['x'] = msg.x
        self.pose['y'] = msg.y
        self.pose['theta'] = msg.theta

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel['x'] = msg.linear.x
        self.cmd_vel['y'] = msg.linear.y
        self.cmd_vel['theta'] = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = ArrowPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
