import math
import struct
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from robot_interfaces.srv import GetSensor2D


class TopographyField(Node):
    """Synthetic terrain field that mixes ridges and trenches."""

    def __init__(self) -> None:
        super().__init__('sensor_topography_2d')

        # Spatial extent and sampling resolution.
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('extent_x', 200.0)
        self.declare_parameter('extent_y', 200.0)
        self.declare_parameter('resolution', 1.0)

        # Publisher configuration.
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_interval', 0.5)

        # Elevation features.
        self.declare_parameter('base_elevation', 25.0)
        self.declare_parameter('ridge_height', 60.0)
        self.declare_parameter('ridge_center_x', 0.0)
        self.declare_parameter('ridge_center_y', 25.0)
        self.declare_parameter('ridge_length_scale', 80.0)
        self.declare_parameter('ridge_width_scale', 8.0)
        self.declare_parameter('ridge_angle_deg', 25.0)

        self.declare_parameter('trench_depth', -40.0)
        self.declare_parameter('trench_center_x', -30.0)
        self.declare_parameter('trench_center_y', -40.0)
        self.declare_parameter('trench_length_scale', 60.0)
        self.declare_parameter('trench_width_scale', 6.0)
        self.declare_parameter('trench_angle_deg', -40.0)

        self.declare_parameter('undulation_amplitude', 8.0)
        self.declare_parameter('undulation_scale_x', 75.0)
        self.declare_parameter('undulation_scale_y', 55.0)

        # Resolve parameter values.
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.extent_x = max(float(self.get_parameter('extent_x').value), 0.1)
        self.extent_y = max(float(self.get_parameter('extent_y').value), 0.1)
        self.resolution = max(float(self.get_parameter('resolution').value), 0.01)

        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_interval = max(float(self.get_parameter('publish_interval').value), 0.01)

        self.base_elevation = float(self.get_parameter('base_elevation').value)

        self.ridge_height = float(self.get_parameter('ridge_height').value)
        self.ridge_center_x = float(self.get_parameter('ridge_center_x').value)
        self.ridge_center_y = float(self.get_parameter('ridge_center_y').value)
        self.ridge_length_scale = max(float(self.get_parameter('ridge_length_scale').value), 1e-6)
        self.ridge_width_scale = max(float(self.get_parameter('ridge_width_scale').value), 1e-6)
        self.ridge_angle = math.radians(float(self.get_parameter('ridge_angle_deg').value))

        self.trench_depth = float(self.get_parameter('trench_depth').value)
        self.trench_center_x = float(self.get_parameter('trench_center_x').value)
        self.trench_center_y = float(self.get_parameter('trench_center_y').value)
        self.trench_length_scale = max(float(self.get_parameter('trench_length_scale').value), 1e-6)
        self.trench_width_scale = max(float(self.get_parameter('trench_width_scale').value), 1e-6)
        self.trench_angle = math.radians(float(self.get_parameter('trench_angle_deg').value))

        self.undulation_amplitude = float(self.get_parameter('undulation_amplitude').value)
        self.undulation_scale_x = max(float(self.get_parameter('undulation_scale_x').value), 1e-6)
        self.undulation_scale_y = max(float(self.get_parameter('undulation_scale_y').value), 1e-6)

        self.points, self.elevations = self._generate_field()

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
        elevations = self._evaluate_elevation(points[:, 0], points[:, 1])
        return points, elevations

    def _oriented_gaussian(self, x: np.ndarray, y: np.ndarray, center_x: float, center_y: float,
                           length_scale: float, width_scale: float, angle_rad: float) -> np.ndarray:
        """Evaluate an oriented anisotropic Gaussian."""
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        dx = x - center_x
        dy = y - center_y
        major = cos_a * dx + sin_a * dy
        minor = -sin_a * dx + cos_a * dy
        major_term = (major / length_scale) ** 2
        minor_term = (minor / width_scale) ** 2
        return np.exp(-0.5 * (major_term + minor_term))

    def _evaluate_elevation(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        ridge = self.ridge_height * self._oriented_gaussian(
            x,
            y,
            self.ridge_center_x,
            self.ridge_center_y,
            self.ridge_length_scale,
            self.ridge_width_scale,
            self.ridge_angle,
        )

        trench = self.trench_depth * self._oriented_gaussian(
            x,
            y,
            self.trench_center_x,
            self.trench_center_y,
            self.trench_length_scale,
            self.trench_width_scale,
            self.trench_angle,
        )

        undulation = self.undulation_amplitude * (
            np.sin((x - self.center_x) / self.undulation_scale_x)
            * np.cos((y - self.center_y) / self.undulation_scale_y)
        )

        return self.base_elevation + ridge + trench + undulation

    def _evaluate_elevation_scalar(self, x: float, y: float) -> float:
        return float(self._evaluate_elevation(np.array([x]), np.array([y]))[0])

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

        elevations = self.elevations.astype(np.float32)
        buffer = bytearray(point_step * len(self.points))
        for index, (x_coord, y_coord) in enumerate(self.points):
            value = float(elevations[index])
            struct.pack_into('ffff', buffer, index * point_step, float(x_coord), float(y_coord), 0.0, value)

        msg.data = bytes(buffer)
        self.publisher.publish(msg)

    def _handle_request_2d(self, request: GetSensor2D.Request, response: GetSensor2D.Response) -> GetSensor2D.Response:
        response.data = self._evaluate_elevation_scalar(request.x, request.y)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopographyField()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
