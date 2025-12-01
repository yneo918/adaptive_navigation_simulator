"""
ROS 2 Node for Synthetic Terrain Field based on Appendix A Scalar Field.

The scalar field S is composed of:
- M1, M2: Maximum (hill) features
- M3: Minimum (valley) feature  
- R1: Linear ridge
- R2: Curved ridge
- T1: Trench
"""

import struct
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from robot_interfaces.srv import GetSensor2D


class TopographyField(Node):
    """Synthetic terrain field based on Appendix A scalar field equations."""

    def __init__(self) -> None:
        super().__init__('sensor_topography_2d')

        # Spatial extent and sampling resolution.
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('extent_x', 600.0)
        self.declare_parameter('extent_y', 600.0)
        self.declare_parameter('resolution', 2.0)

        # Publisher configuration.
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_interval', 0.5)

        # =================================================================
        # Scalar Field Parameters from Appendix A
        # =================================================================

        # Table A-1: Maximum/Minimum parameters
        # M1: Global Maximum
        self.declare_parameter('m1_height', 50.0)
        self.declare_parameter('m1_rolloff', 0.0001)
        self.declare_parameter('m1_x', 150.0)
        self.declare_parameter('m1_y', 150.0)

        # M2: Local Maximum
        self.declare_parameter('m2_height', 20.0)
        self.declare_parameter('m2_rolloff', 0.0001)
        self.declare_parameter('m2_x', 0.0)
        self.declare_parameter('m2_y', 150.0)

        # M3: Global Minimum
        self.declare_parameter('m3_height', -25.0)
        self.declare_parameter('m3_rolloff', 0.0001)
        self.declare_parameter('m3_x', -100.0)
        self.declare_parameter('m3_y', -150.0)

        # Table A-2: Linear Ridge (R1) parameters
        self.declare_parameter('r1_height', 30.0)
        self.declare_parameter('r1_end_rolloff', 0.007)
        self.declare_parameter('r1_rolloff', 0.0006)
        self.declare_parameter('r1_rolloff_x', -25.0)
        self.declare_parameter('r1_x', 150.0)
        self.declare_parameter('r1_y', -75.0)

        # Table A-3: Curved Ridge (R2) parameters
        self.declare_parameter('r2_height', 25.0)
        self.declare_parameter('r2_end_rolloff', 0.007)
        self.declare_parameter('r2_rolloff', 0.005)
        self.declare_parameter('r2_x', -50.0)
        self.declare_parameter('r2_y', -75.0)
        self.declare_parameter('r2_xc', -200.0)
        self.declare_parameter('r2_yc', 250.0)
        self.declare_parameter('r2_radius', 200.0)

        # Table A-4: Trench (T1) parameters
        self.declare_parameter('t1_height', -15.0)
        self.declare_parameter('t1_end_rolloff', 0.01)
        self.declare_parameter('t1_rolloff', 0.03)
        self.declare_parameter('t1_x', 0.0)
        self.declare_parameter('t1_x1', -100.0)
        self.declare_parameter('t1_y1', -150.0)
        self.declare_parameter('t1_x2', 75.0)
        self.declare_parameter('t1_y2', 150.0)

        # Resolve spatial parameters.
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.extent_x = max(float(self.get_parameter('extent_x').value), 0.1)
        self.extent_y = max(float(self.get_parameter('extent_y').value), 0.1)
        self.resolution = max(float(self.get_parameter('resolution').value), 0.01)

        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_interval = max(float(self.get_parameter('publish_interval').value), 0.01)

        # Resolve M1 parameters (Global Maximum).
        self.m1_height = float(self.get_parameter('m1_height').value)
        self.m1_rolloff = float(self.get_parameter('m1_rolloff').value)
        self.m1_x = float(self.get_parameter('m1_x').value)
        self.m1_y = float(self.get_parameter('m1_y').value)

        # Resolve M2 parameters (Local Maximum).
        self.m2_height = float(self.get_parameter('m2_height').value)
        self.m2_rolloff = float(self.get_parameter('m2_rolloff').value)
        self.m2_x = float(self.get_parameter('m2_x').value)
        self.m2_y = float(self.get_parameter('m2_y').value)

        # Resolve M3 parameters (Global Minimum).
        self.m3_height = float(self.get_parameter('m3_height').value)
        self.m3_rolloff = float(self.get_parameter('m3_rolloff').value)
        self.m3_x = float(self.get_parameter('m3_x').value)
        self.m3_y = float(self.get_parameter('m3_y').value)

        # Resolve R1 parameters (Linear Ridge).
        self.r1_height = float(self.get_parameter('r1_height').value)
        self.r1_end_rolloff = float(self.get_parameter('r1_end_rolloff').value)
        self.r1_rolloff = float(self.get_parameter('r1_rolloff').value)
        self.r1_rolloff_x = float(self.get_parameter('r1_rolloff_x').value)
        self.r1_x = float(self.get_parameter('r1_x').value)
        self.r1_y = float(self.get_parameter('r1_y').value)

        # Resolve R2 parameters (Curved Ridge).
        self.r2_height = float(self.get_parameter('r2_height').value)
        self.r2_end_rolloff = float(self.get_parameter('r2_end_rolloff').value)
        self.r2_rolloff = float(self.get_parameter('r2_rolloff').value)
        self.r2_x = float(self.get_parameter('r2_x').value)
        self.r2_y = float(self.get_parameter('r2_y').value)
        self.r2_xc = float(self.get_parameter('r2_xc').value)
        self.r2_yc = float(self.get_parameter('r2_yc').value)
        self.r2_radius = float(self.get_parameter('r2_radius').value)

        # Resolve T1 parameters (Trench).
        self.t1_height = float(self.get_parameter('t1_height').value)
        self.t1_end_rolloff = float(self.get_parameter('t1_end_rolloff').value)
        self.t1_rolloff = float(self.get_parameter('t1_rolloff').value)
        self.t1_x = float(self.get_parameter('t1_x').value)
        self.t1_x1 = float(self.get_parameter('t1_x1').value)
        self.t1_y1 = float(self.get_parameter('t1_y1').value)
        self.t1_x2 = float(self.get_parameter('t1_x2').value)
        self.t1_y2 = float(self.get_parameter('t1_y2').value)

        # Generate field data.
        self.points, self.elevations = self._generate_field()
        self.point_step = struct.calcsize('ffff')
        self.pointcloud_msg = self._create_pointcloud_message()

        # ROS 2 interfaces.
        self.publisher = self.create_publisher(PointCloud2, 'sensor_field/points', 10)
        self.timer = self.create_timer(publish_interval, self._publish_pointcloud)
        self.service = self.create_service(GetSensor2D, 'get_sensor', self._handle_request_2d)

    def _generate_field(self) -> Tuple[np.ndarray, np.ndarray]:
        """Generate the scalar field grid points and elevation values."""
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

    def _compute_maximum_minimum(self, x: np.ndarray, y: np.ndarray,
                                  m_height: float, m_rolloff: float,
                                  x_m: float, y_m: float) -> np.ndarray:
        """
        Equation A-2: Maximum (hill) or Minimum (valley) feature.

        M = m_height / (m_rolloff * ((x - x_m)^2 + (y - y_m)^2) + 1)
        """
        return m_height / (m_rolloff * ((x - x_m)**2 + (y - y_m)**2) + 1)

    def _compute_linear_ridge(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Equation A-3: Linear ridge feature (R1).

        R1 = r1_height / (((r1_end_rolloff * dy)^4 + 1) * ((r1_rolloff * (r1_rolloff_x * dx + dy))^2 + 1))
        """
        dx = x - self.r1_x
        dy = y - self.r1_y

        term1 = (self.r1_end_rolloff * dy)**4 + 1
        term2 = (self.r1_rolloff * (self.r1_rolloff_x * dx + dy))**2 + 1

        return self.r1_height / (term1 * term2)

    def _compute_curved_ridge(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Equation A-4: Curved ridge feature (R2).

        R2 = r2_height / (((r2_end_rolloff * dist)^4 + 1) * (r2_rolloff * (dist_c - r_r2)^2 + 1))
        """
        dx = x - self.r2_x
        dy = y - self.r2_y
        dxc = x - self.r2_xc
        dyc = y - self.r2_yc

        dist_from_center = np.sqrt(dx**2 + dy**2)
        dist_from_circle_center = np.sqrt(dxc**2 + dyc**2)

        term1 = (self.r2_end_rolloff * dist_from_center)**4 + 1
        term2 = self.r2_rolloff * (dist_from_circle_center - self.r2_radius)**2 + 1

        return self.r2_height / (term1 * term2)

    def _compute_trench(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Equations A-5 to A-8: Trench feature (T1).

        T = t_height / (((t_end_rolloff * dx)^4 + 1) * ((t_rolloff * d_t)^2 + 1))
        """
        # Equation A-7, A-8: Line direction.
        t_dx = self.t1_x2 - self.t1_x1
        t_dy = self.t1_y2 - self.t1_y1

        # dx for end rolloff calculation.
        dx = x - self.t1_x

        # Equation A-6: Distance from point to trench line.
        numerator = np.abs(t_dy * x - t_dx * y + self.t1_x2 * self.t1_y1 - self.t1_y2 * self.t1_x1)
        denominator = np.sqrt(t_dy**2 + t_dx**2)
        d_t = numerator / denominator

        # Equation A-5: Trench value.
        term1 = (self.t1_end_rolloff * dx)**4 + 1
        term2 = (self.t1_rolloff * d_t)**2 + 1

        return self.t1_height / (term1 * term2)

    def _evaluate_elevation(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Equation A-1: Compute the complete scalar field S.

        S = M1 + M2 + M3 + R1 + R2 + T1
        """
        # Maximum/Minimum features (Equation A-2).
        m1 = self._compute_maximum_minimum(x, y, self.m1_height, self.m1_rolloff, self.m1_x, self.m1_y)
        m2 = self._compute_maximum_minimum(x, y, self.m2_height, self.m2_rolloff, self.m2_x, self.m2_y)
        m3 = self._compute_maximum_minimum(x, y, self.m3_height, self.m3_rolloff, self.m3_x, self.m3_y)

        # Ridge features.
        r1 = self._compute_linear_ridge(x, y)
        r2 = self._compute_curved_ridge(x, y)

        # Trench feature.
        t1 = self._compute_trench(x, y)

        return m1 + m2 + m3 + r1 + r2 + t1

    def _evaluate_elevation_scalar(self, x: float, y: float) -> float:
        """Evaluate elevation at a single point."""
        return float(self._evaluate_elevation(np.array([x]), np.array([y]))[0])

    def _create_pointcloud_message(self) -> PointCloud2:
        """Create the PointCloud2 message for visualization."""
        msg = PointCloud2()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(self.points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = self.point_step
        msg.row_step = self.point_step * len(self.points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        elevations = self.elevations.astype(np.float32, copy=False)
        buffer = bytearray(self.point_step * len(self.points))
        for index, (x_coord, y_coord) in enumerate(self.points):
            value = float(elevations[index])
            struct.pack_into('ffff', buffer, index * self.point_step, float(x_coord), float(y_coord), value*2,  value)

        msg.data = bytes(buffer)
        return msg

    def _publish_pointcloud(self) -> None:
        """Publish the pointcloud message with updated timestamp."""
        self.pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pointcloud_msg)

    def _handle_request_2d(self, request: GetSensor2D.Request, response: GetSensor2D.Response) -> GetSensor2D.Response:
        """Handle service request for elevation at a specific point."""
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
