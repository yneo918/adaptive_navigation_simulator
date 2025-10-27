import rclpy
import math
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from pioneer_interfaces.msg import ClusterInfo

from gui_package.my_ros_module import PubSubManager

from .Controller import Controller

def main(args=None):
    """Main entry point for the controller node"""
    rclpy.init(args=args)
    
    try:
        controller = Controller("cluster_controller")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in controller: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()