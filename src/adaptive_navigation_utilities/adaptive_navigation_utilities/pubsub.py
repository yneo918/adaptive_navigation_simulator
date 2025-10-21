import rclpy
from rclpy.node import Node

class PubSubManager:
    def __init__(self, node=None):
        self.node = node
        self._subscribers = [[], {}]  # [list of subscriber objects, dict of topic_name : list index]
        self._publishers = [[], {}]  # [list of subscriber objects, dict of topic_name : list index]
        
    def create_subscription(self, msg_type, topic_name, callback, qos=10, **kwargs):
        self._subscribers[0].append(
            self.node.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos,
                **kwargs
            )
        )
        self._subscribers[1][topic_name] = len(self._subscribers[0]) - 1

    def create_publisher(self, msg_type, topic_name, qos=10, **kwargs):
        self._publishers[0].append(
            self.node.create_publisher(
                msg_type,
                topic_name,
                qos,
                **kwargs
            )
        )
        self._publishers[1][topic_name] = len(self._publishers[0]) - 1
    
    def publish(self, topic_name, msg):
        if topic_name in self._publishers[1]:
            self._publishers[0][self._publishers[1][topic_name]].publish(msg)
        else:
            print(f"Publisher for {topic_name} not found.")
            raise ValueError(f"Publisher for {topic_name} not found.")