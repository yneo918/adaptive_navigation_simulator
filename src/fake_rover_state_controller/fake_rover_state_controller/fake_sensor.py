import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Float64
from robot_interfaces.srv import GetSensor2D

from .my_ros_module import PubSubManager

UPDATE_RATE = 10.0 # Hz

class FakeSensor(Node):

    def __init__(self):
        super().__init__('fake_rover')
        self.pubsub = PubSubManager(self)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p0"),
                ('prefix', "/sim"),
                ('sensor_name', "sensor"),
                ('sensor_service_name', "get_sensor")
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        self.robot_id = self.get_parameter('robot_id').value
        self.prefix = self.get_parameter('prefix').value
        self.sensor_msg_name = self.get_parameter('sensor_name').value
        self.sensor_service_name = self.get_parameter("sensor_service_name").value

        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.sensor = None

        self.pubsub.create_subscription(Pose2D, f'{self.prefix}/{self.robot_id}/pose2D', self.position_callback, 10)
        self.pubsub.create_publisher(Float64, f'{self.prefix}/{self.robot_id}/{self.sensor_msg_name}', 5)
        
        timer_period = 1/ UPDATE_RATE
        self.pub_timer = self.create_timer(timer_period, self.timer_callback)

        self.client = self.create_client(GetSensor2D, 'get_sensor')
        if self.client.wait_for_service(timeout_sec=1.0):
            self.sensor_timer = self.create_timer(timer_period, self.check_sensor)

    def position_callback(self, msg):
        self.position = {'x':msg.x, 'y':msg.y, 'theta':msg.theta}
        self.check_sensor()
        
    def timer_callback(self):
        if self.sensor is not None:
            self.publish_sensor()
    
    def publish_sensor(self):
        sensor_msg = Float64()
        sensor_msg.data = self.sensor
        self.pubsub.publish(f'{self.prefix}/{self.robot_id}/{self.sensor_msg_name}', sensor_msg)

    
    def check_sensor(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        request = GetSensor2D.Request()
        request.x = self.position['x']
        request.y = self.position['y']
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.sensor = response.data
            #self.get_logger().info(f"Received sensor: {self.sensor}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    fake_sensor = FakeSensor()

    rclpy.spin(fake_sensor)

    fake_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()