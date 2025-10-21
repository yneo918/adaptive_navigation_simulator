import math
import struct
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from robot_interfaces.srv import GetSensor2D


class SensorField(Node):
    """Synthetic 2D sensor field that forms an elliptical hill."""

    def __init__(self) -> None:
        super().__init__('sensor_sample_2d')

        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('sigma_x', 25.0)
        self.declare_parameter('sigma_y', 10.0)
        self.declare_parameter('peak_height', 100.0)
        self.declare_parameter('extent_x', 100.0)
        self.declare_parameter('extent_y', 100.0)
        self.declare_parameter('resolution', 1.0)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_interval', 0.5)

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.sigma_x = max(float(self.get_parameter('sigma_x').value), 1e-6)
        self.sigma_y = max(float(self.get_parameter('sigma_y').value), 1e-6)
        self.peak_height = float(self.get_parameter('peak_height').value)
        self.extent_x = max(float(self.get_parameter('extent_x').value), 0.1)
        self.extent_y = max(float(self.get_parameter('extent_y').value), 0.1)
        self.resolution = max(float(self.get_parameter('resolution').value), 0.01)
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_interval = max(float(self.get_parameter('publish_interval').value), 0.01)

        self.points, self.heights = self._generate_field()

        self.publisher = self.create_publisher(PointCloud2, 'sensor_field/points', 10)
        self.timer = self.create_timer(publish_interval, self._publish_pointcloud)
        self.service = self.create_service(GetSensor2D, 'get_sensor', self._handle_request_2d)

    def _generate_field(self) -> Tuple[np.ndarray, np.ndarray]:
        x_start = self.center_x - self.extent_x * 0.5
        x_stop = self.center_x + self.extent_x * 0.5
        y_start = self.center_y - self.extent_y * 0.5
        y_stop = self.center_y + self.extent_y * 0.5

        x_coords = np.arange(x_start, x_stop + self.resolution * 0.5, self.resolution, dtype=np.float64)
        y_coords = np.arange(y_start, y_stop + self.resolution * 0.5, self.resolution, dtype=np.float64)
        grid_x, grid_y = np.meshgrid(x_coords, y_coords)

        points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
        heights = self._evaluate_height(points[:, 0], points[:, 1])
        return points, heights

    def _evaluate_height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        dx = (x - self.center_x) / self.sigma_x
        dy = (y - self.center_y) / self.sigma_y
        exponent = -0.5 * (dx * dx + dy * dy)
        return self.peak_height * np.exp(exponent)

    def _evaluate_height_scalar(self, x: float, y: float) -> float:
        dx = (x - self.center_x) / self.sigma_x
        dy = (y - self.center_y) / self.sigma_y
        exponent = -0.5 * (dx * dx + dy * dy)
        return float(self.peak_height * math.exp(exponent))

    def _publish_pointcloud(self) -> None:
        msg = PointCloud2()
        header = self.get_clock().now().to_msg()
        msg.header.stamp = header
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(self.points)
        msg.is_dense = False
        msg.is_bigendian = False

        point_step = struct.calcsize('ffff')
        msg.point_step = point_step
        msg.row_step = point_step * len(self.points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        heights = self.heights.astype(np.float32)
        buffer = bytearray(point_step * len(self.points))
        for index, (x, y) in enumerate(self.points):
            z_value = float(heights[index])
            struct.pack_into('ffff', buffer, index * point_step, float(x), float(y), 0.0, z_value)

        msg.data = bytes(buffer)
        self.publisher.publish(msg)

    def _handle_request_2d(self, request: GetSensor2D.Request, response: GetSensor2D.Response) -> GetSensor2D.Response:
        response.data = self._evaluate_height_scalar(request.x, request.y)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorField()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
