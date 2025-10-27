import rclpy
from rclpy.node import Node

import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D

from .my_ros_module import PubSubManager

UPDATE_RATE = 0.1


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('jointstate_publisher')
        self.pubsub = PubSubManager(self)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p0")
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        self.robot_id = self.get_parameter('robot_id').value

        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.position_desired = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.joint_names = ['w_to_x', 'x_to_y', 'y_to_t']

        # Subscribe to pose data from different sources
        self.pubsub.create_subscription(Pose2D, f'/{self.robot_id}/pose2D', self.pose_callback, 10)
        self.pubsub.create_subscription(Pose2D, f'/{self.robot_id}/desiredPose2D', self.desired_pose_callback, 10)
        
        # Publish joint states for visualization
        self.pubsub.create_publisher(JointState, f'/{self.robot_id}/joint_states', 10)
        self.pubsub.create_publisher(JointState, f'/{self.robot_id}desired/joint_states', 10)
        
        timer_period = UPDATE_RATE
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

    def publish_joint_states(self):
        t = time.time()
        
        # Publish simulated joint states
        jointstates_msg = JointState()
        jointstates_msg.header.stamp.sec = int(t // 1)
        jointstates_msg.header.stamp.nanosec = int((t % 1) * 1e9)
        jointstates_msg.name = [f'{joint_name}' for joint_name in self.joint_names]
        jointstates_msg.position = [
            self.position['x'], self.position['y'], self.position['theta']
        ]
        self.pubsub.publish(f'/{self.robot_id}/joint_states', jointstates_msg)

        # Publish desired joint states
        jointstates_msg.position = [
            self.position_desired['x'], self.position_desired['y'], self.position_desired['theta']
        ]
        self.pubsub.publish(f'/{self.robot_id}desired/joint_states', jointstates_msg)

    def pose_callback(self, msg):
        self.position.update({'x': msg.x, 'y': msg.y, 'theta': msg.theta})


    def desired_pose_callback(self, msg):
        self.position_desired.update({'x': msg.x, 'y': msg.y, 'theta': msg.theta})


def main(args=None):
    rclpy.init(args=args)

    jointstate_publisher = JointStatePublisher()

    rclpy.spin(jointstate_publisher)

    jointstate_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()