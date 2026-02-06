"""
Simulation clock publisher for time acceleration.

Publishes /clock topic at configurable time_scale to enable
synchronized time acceleration across all nodes using use_sim_time.
"""

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

# Time scale constraints
MIN_TIME_SCALE = 0.01
MAX_TIME_SCALE = 1000.0

# Clock publish rate configuration
DEFAULT_PUBLISH_RATE_HZ = 100.0
MAX_PUBLISH_RATE_HZ = 10000.0


class ClockPublisher(Node):
    """Publishes simulation time to /clock topic with configurable acceleration."""

    def __init__(self) -> None:
        super().__init__('clock_publisher')

        # Declare parameters
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('publish_rate_hz', DEFAULT_PUBLISH_RATE_HZ)

        # Get parameters
        self.time_scale = float(self.get_parameter('time_scale').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # Validate time_scale range
        if not (MIN_TIME_SCALE <= self.time_scale <= MAX_TIME_SCALE):
            self.get_logger().error(
                f'Invalid time_scale: {self.time_scale}. '
                f'Must be between {MIN_TIME_SCALE} and {MAX_TIME_SCALE}'
            )
            raise ValueError(f'time_scale out of range: {self.time_scale}')

        # Validate publish_rate_hz
        if publish_rate_hz <= 0.0:
            self.get_logger().error(f'Invalid publish_rate_hz: {publish_rate_hz}. Must be > 0.')
            raise ValueError('publish_rate_hz must be positive')

        # Cap publish rate at maximum
        publish_rate_hz = min(publish_rate_hz, MAX_PUBLISH_RATE_HZ)

        # Warn if time_scale is very high
        if self.time_scale > 100.0:
            self.get_logger().warn(
                f'⚠ time_scale={self.time_scale} is very high. '
                f'Clock publish rate: {publish_rate_hz:.0f} Hz. '
                f'High CPU usage expected.'
            )

        # Initialize simulation time (starts at 0)
        self.sim_time_ns = 0

        # Create /clock publisher
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        # Create timer for publishing clock
        timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(timer_period, self._publish_clock)

        self.get_logger().info(
            f'Clock publisher started: time_scale={self.time_scale}, '
            f'publish_rate={publish_rate_hz}Hz'
        )

    def _publish_clock(self) -> None:
        """Publish current simulation time to /clock topic."""
        # Increment simulation time based on timer period and time_scale
        timer_period = self.timer.timer_period_ns / 1e9
        dt_sim = timer_period * self.time_scale
        self.sim_time_ns += int(dt_sim * 1e9)

        # Create and publish clock message
        clock_msg = Clock()
        clock_msg.clock.sec = self.sim_time_ns // 1_000_000_000
        clock_msg.clock.nanosec = self.sim_time_ns % 1_000_000_000
        self.clock_pub.publish(clock_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
