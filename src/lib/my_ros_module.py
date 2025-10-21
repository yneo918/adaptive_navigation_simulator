import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix

from math import sin, cos, asin, atan2, sqrt, degrees, pi, radians
from typing import Dict, List, Any, Callable, Optional


class JoyBase(Node):
    def __init__(self, node_name='joy_node'):
        super().__init__(node_name)
        self.button_dict = { 
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "LB": 4,
            "RB": 5,
            "BACK": 6,
            "START": 7,
            "POWER": 8,
            "LS": 9,
            "RS": 10
        }

        self.axis_dict = {
            "LX": 0,
            "LY": 1,
            "LT": 2,
            "RX": 3,
            "RY": 4,
            "RT": 5,
            "cross_lr": 6,
            "cross_ud": 7
        }
        self.pubsub = PubSubManager(self)

        self.pubsub.create_subscription(Joy, 'joy', self.joy_callback)

        self.prev_joy = Joy()
        #self.lx_axisN = self.axis_dict.get("LY")
    
    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)
    
    def joy_toggle(self, joy_msg):
        if len(self.prev_joy.buttons) != len(joy_msg.buttons):
            self.prev_joy = joy_msg

        ret = [0] * len(joy_msg.buttons)
        for i in range(len(joy_msg.buttons)):
            ret[i] = self.prev_joy.buttons[i] - joy_msg.buttons[i]
        self.prev_joy = joy_msg
        return ret


iniDesiredCoor = [37.35228, -121.941788] # Garage

class NavNode(Node):
    STATUS_ROVER = {
        "IDLE": 0,
        "RUNNING": 1,
        "STOP": 2
    }
    R  = 6373.0
    def __init__(self, node_name='nav_node'):
        super().__init__(node_name)
        self.pubsub = PubSubManager(self)
    
    def get_bearing_distance(self, lat0, lon0, lat1, lon1):
        delta_lat = radians(lat1) - radians(lat0)
        delta_lon = radians(lon1) - radians(lon0)
        cos_lat1 = cos(radians(lat1))
        cos_lat0 = cos(radians(lat0))
        # bearing
        bearingX = cos_lat1 * sin(delta_lon)
        bearingY = cos_lat0 * sin(radians(lat1)) - sin(radians(lat0)) * cos_lat1 * cos(delta_lon)
        yaw = -atan2(bearingX,bearingY)
        if yaw<0:
            yaw += 2*pi
        bearing = degrees(yaw)

        # distance
        a = sin(delta_lat/2)**2 + cos_lat1 * cos_lat0 * sin(delta_lon/2)**2
        c = 2* atan2(sqrt(a), sqrt(1-a))
        dist = self.R*c*1000
        return bearing, dist
    
    def get_target_pos(self, lat0, lon0, bearing, dist):
        lat0 = radians(lat0)
        lon0 = radians(lon0)
        bearing = radians(bearing)
        dist = dist/1000
        d = dist/self.R
        lat1 = asin(sin(lat0)*cos(d) + cos(lat0)*sin(d)*cos(bearing))
        lon1 = lon0 + atan2(sin(bearing)*sin(d)*cos(lat0), cos(d)-sin(lat0)*sin(lat1))
        return degrees(lat1), degrees(lon1)
    
    def dict_reverse_lookup(d, value):
        for key, val in d.items():
            if val == value:
                return key
        return None


class PubSubManager:
    """Manager class for ROS2 publishers and subscribers"""
    
    def __init__(self, node: Node):
        """
        Initialize PubSubManager
        
        Args:
            node: ROS2 node instance
        """
        if not isinstance(node, Node):
            raise TypeError("node must be an instance of rclpy.node.Node")
        
        self.node = node
        self._subscribers: Dict[str, Any] = {}
        self._publishers: Dict[str, Any] = {}
        
    def create_subscription(self, 
                         msg_type: type, 
                         topic_name: str, 
                         callback: Callable, 
                         qos: int = 10, 
                         **kwargs) -> None:
        """
        Create a subscriber
        
        Args:
            msg_type: Message type
            topic_name: Topic name
            callback: Callback function
            qos: QoS setting
            **kwargs: Additional options
            
        Raises:
            ValueError: If subscriber for the same topic already exists
        """
        if topic_name in self._subscribers:
            raise ValueError(f"Subscriber for topic '{topic_name}' already exists")
        
        try:
            subscriber = self.node.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos,
                **kwargs
            )
            self._subscribers[topic_name] = subscriber
            self.node.get_logger().info(f"Created subscriber for topic: {topic_name}")
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to create subscriber for {topic_name}: {e}")
            raise

    def create_publisher(self, 
                        msg_type: type, 
                        topic_name: str, 
                        qos: int = 10, 
                        **kwargs) -> None:
        """
        Create a publisher
        
        Args:
            msg_type: Message type
            topic_name: Topic name
            qos: QoS setting
            **kwargs: Additional options
            
        Raises:
            ValueError: If publisher for the same topic already exists
        """
        if topic_name in self._publishers:
            raise ValueError(f"Publisher for topic '{topic_name}' already exists")
        
        try:
            publisher = self.node.create_publisher(
                msg_type,
                topic_name,
                qos,
                **kwargs
            )
            self._publishers[topic_name] = publisher
            self.node.get_logger().info(f"Created publisher for topic: {topic_name}")
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to create publisher for {topic_name}: {e}")
            raise
    
    def publish(self, topic_name: str, msg: Any) -> None:
        """
        Publish a message
        
        Args:
            topic_name: Topic name
            msg: Message to publish
            
        Raises:
            ValueError: If publisher for the specified topic doesn't exist
        """
        if topic_name not in self._publishers:
            error_msg = f"Publisher for topic '{topic_name}' not found"
            self.node.get_logger().error(error_msg)
            raise ValueError(error_msg)
        
        try:
            self._publishers[topic_name].publish(msg)
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to publish to {topic_name}: {e}")
            raise
    
    def has_subscriber(self, topic_name: str) -> bool:
        """Check if subscriber for the specified topic exists"""
        return topic_name in self._subscribers
    
    def has_publisher(self, topic_name: str) -> bool:
        """Check if publisher for the specified topic exists"""
        return topic_name in self._publishers
    
    def get_subscriber_topics(self) -> List[str]:
        """Get list of registered subscriber topic names"""
        return list(self._subscribers.keys())
    
    def get_publisher_topics(self) -> List[str]:
        """Get list of registered publisher topic names"""
        return list(self._publishers.keys())
    
    def remove_subscriber(self, topic_name: str) -> bool:
        """
        Remove a subscriber
        
        Args:
            topic_name: Topic name to remove
            
        Returns:
            True if removal was successful, False if topic doesn't exist
        """
        if topic_name in self._subscribers:
            # ROS2 automatically handles subscriber cleanup
            del self._subscribers[topic_name]
            self.node.get_logger().info(f"Removed subscriber for topic: {topic_name}")
            return True
        return False
    
    def remove_publisher(self, topic_name: str) -> bool:
        """
        Remove a publisher
        
        Args:
            topic_name: Topic name to remove
            
        Returns:
            True if removal was successful, False if topic doesn't exist
        """
        if topic_name in self._publishers:
            # ROS2 automatically handles publisher cleanup
            del self._publishers[topic_name]
            self.node.get_logger().info(f"Removed publisher for topic: {topic_name}")
            return True
        return False
    
    def clear_all(self) -> None:
        """Clear all publishers and subscribers"""
        self._subscribers.clear()
        self._publishers.clear()
        self.node.get_logger().info("Cleared all publishers and subscribers")
    
    def get_stats(self) -> Dict[str, int]:
        """Get statistics information"""
        return {
            'subscribers': len(self._subscribers),
            'publishers': len(self._publishers)
        }
    
    
def main(args=None):
    rclpy.init(args=args)
    joy_handle = JoyBase()
    rclpy.spin(joy_handle)
    joy_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
