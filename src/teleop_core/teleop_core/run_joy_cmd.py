"""Simple joystick command node without GUI."""

import rclpy
from std_msgs.msg import Int16

from .joy_cmd_base import JoyCmdBase
from .constants import DEFAULT_QOS


class JoyCmd(JoyCmdBase):
    """Joystick command node for basic operation without GUI."""

    def __init__(self):
        super().__init__('joy_cmd')
        self._setup_additional_publishers()

    def _setup_additional_publishers(self) -> None:
        """Setup additional publishers."""
        self.pubsub.create_publisher(Int16, '/joy/angle_sel', DEFAULT_QOS)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        joy_cmd = JoyCmd()
        rclpy.spin(joy_cmd)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'joy_cmd' in locals():
            joy_cmd.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()