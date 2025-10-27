import csv
import math
import struct
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from robot_interfaces.srv import GetSensor2D, GetSensor3D


EARTH_RADIUS_M = 6_371_000.0


class CsvSensorFieldPublisher(Node):
    """Publish sensor readings from CSV data and serve nearest-neighbour lookups."""

    def __init__(self) -> None:
        super().__init__('csv_sensor_field')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('pointcloud_topic', 'sensor_field/points')
        self.declare_parameter('publish_interval', 0.5)
        self.declare_parameter('column_x', 'x')
        self.declare_parameter('column_y', 'y')
        self.declare_parameter('column_z', '')
        self.declare_parameter('column_data', 'a')
        self.declare_parameter('interpret_latlon', False)
        self.declare_parameter('distance_scale', 1.0)

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        if not csv_path:
            raise ValueError('Parameter "csv_path" must be set to a CSV file path.')

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        publish_interval = self.get_parameter('publish_interval').get_parameter_value().double_value

        column_x = self.get_parameter('column_x').get_parameter_value().string_value
        column_y = self.get_parameter('column_y').get_parameter_value().string_value
        column_z_param = self.get_parameter('column_z').get_parameter_value().string_value
        column_data = self.get_parameter('column_data').get_parameter_value().string_value

        if not column_x or not column_y or not column_data:
            raise ValueError('Parameters "column_x", "column_y", and "column_data" must be set to column names.')

        column_z = column_z_param if column_z_param else None

        interpret_latlon = self.get_parameter('interpret_latlon').get_parameter_value().bool_value
        distance_scale = self.get_parameter('distance_scale').get_parameter_value().double_value
        if distance_scale <= 0.0:
            raise ValueError('Parameter "distance_scale" must be a positive value.')

        points, values, dimension, latlon_meta = self._load_csv(
            Path(csv_path),
            column_x,
            column_y,
            column_z,
            column_data,
            interpret_latlon,
            distance_scale,
        )
        self.dimension = dimension
        self.frame_id = frame_id
        self.points = points
        self.values = values
        self.latlon_meta = latlon_meta
        self.latlon_enabled = latlon_meta is not None
        self.lookup_table: Dict[Tuple[float, ...], float] = {
            self._make_key(point): float(value)
            for point, value in zip(self.points, self.values)
        }

        self.publisher = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self.timer = self.create_timer(max(publish_interval, 0.01), self._publish_pointcloud)

        self.column_names = {
            'x': column_x,
            'y': column_y,
            'z': column_z,
            'data': column_data,
        }

        if self.dimension == 2:
            self.service = self.create_service(GetSensor2D, 'get_sensor', self._handle_request_2d)
            self.get_logger().info(
                f'Loaded 2D sensor field from {csv_path} with {len(self.points)} samples '
                f'(columns x={column_x}, y={column_y}, data={column_data}).'
            )
        else:
            self.service = self.create_service(GetSensor3D, 'get_sensor', self._handle_request_3d)
            self.get_logger().info(
                f'Loaded 3D sensor field from {csv_path} with {len(self.points)} samples '
                f'(columns x={column_x}, y={column_y}, z={column_z}, data={column_data}).'
            )

        if self.latlon_enabled:
            origin_lat = math.degrees(self.latlon_meta['origin_lat_rad'])
            origin_lon = math.degrees(self.latlon_meta['origin_lon_rad'])
            self.get_logger().info(
                f'Interpreting x/y as latitude/longitude (origin {origin_lat:.6f}°, '
                f'{origin_lon:.6f}°, scale={self.latlon_meta["scale"]:.6f}).'
            )

    def _load_csv(
        self,
        path: Path,
        column_x: str,
        column_y: str,
        column_z: Optional[str],
        column_data: str,
        interpret_latlon: bool,
        distance_scale: float,
    ) -> Tuple[np.ndarray, np.ndarray, int, Optional[Dict[str, float]]]:
        if not path.exists():
            raise FileNotFoundError(f'CSV file not found: {path}')

        points: list[Tuple[float, ...]] = []
        values: list[float] = []

        with path.open(newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            if reader.fieldnames is None:
                raise ValueError('CSV file must contain a header row with column names.')

            fieldnames = reader.fieldnames
            missing = [name for name in (column_x, column_y, column_data) if name not in fieldnames]
            if missing:
                raise ValueError(
                    f'CSV file {path} does not contain required columns: {", ".join(missing)}'
                )

            z_field: Optional[str] = None
            if column_z:
                if column_z not in fieldnames:
                    raise ValueError(f'CSV file {path} does not contain z column "{column_z}".')
                z_field = column_z

            for row in reader:
                if not row:
                    continue
                try:
                    x_val = float(row[column_x])
                    y_val = float(row[column_y])
                    if z_field:
                        z_val = float(row[z_field])
                        point: Tuple[float, ...] = (x_val, y_val, z_val)
                    else:
                        point = (x_val, y_val)
                    value = float(row[column_data])
                except (TypeError, ValueError, KeyError):
                    continue

                points.append(point)
                values.append(value)

        if not points:
            raise ValueError(f'CSV file {path} does not contain valid numeric rows for the requested columns.')

        points_array = np.asarray(points, dtype=np.float64)
        values_array = np.asarray(values, dtype=np.float64)
        dimension = points_array.shape[1]
        if dimension not in (2, 3):
            raise ValueError('Only 2D or 3D sensor data are supported.')

        latlon_meta: Optional[Dict[str, float]] = None
        if interpret_latlon:
            lat_deg = points_array[:, 0]
            lon_deg = points_array[:, 1]
            lat_rad = np.radians(lat_deg)
            lon_rad = np.radians(lon_deg)
            lat0 = float(lat_rad[0])
            lon0 = float(lon_rad[0])
            scale = float(distance_scale)
            avg_lat = 0.5 * (lat_rad + lat0)
            delta_lat = lat_rad - lat0
            delta_lon = lon_rad - lon0
            north = - EARTH_RADIUS_M * delta_lat * scale
            east = EARTH_RADIUS_M * np.cos(avg_lat) * delta_lon * scale
            converted_xy = np.column_stack((north, east))
            if dimension == 2:
                points_array = converted_xy
            else:
                points_array = np.column_stack((converted_xy, points_array[:, 2:]))
            latlon_meta = {
                'origin_lat_rad': lat0,
                'origin_lon_rad': lon0,
                'scale': scale,
            }

        return points_array, values_array, dimension, latlon_meta

    def _publish_pointcloud(self) -> None:
        header = self.get_clock().now().to_msg()
        point_step = struct.calcsize('ffff')

        if self.dimension == 2:
            padded_points = np.column_stack((self.points, np.zeros(len(self.points))))
        else:
            padded_points = self.points

        intensity = self.values.astype(np.float32)
        buffer = bytearray(point_step * len(padded_points))
        for index, (x, y, z) in enumerate(padded_points):
            struct.pack_into('ffff', buffer, index * point_step, float(x), float(y), intensity[index], intensity[index])

        msg = PointCloud2()
        msg.header.stamp = header
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(padded_points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(padded_points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = bytes(buffer)

        self.publisher.publish(msg)

    def _handle_request_2d(self, request: GetSensor2D.Request, response: GetSensor2D.Response) -> GetSensor2D.Response:
        query = np.array([request.x, request.y], dtype=np.float64)
        response.data = float(self._lookup_value(query))
        return response

    def _handle_request_3d(self, request: GetSensor3D.Request, response: GetSensor3D.Response) -> GetSensor3D.Response:
        query = np.array([request.x, request.y, request.z], dtype=np.float64)
        response.data = float(self._lookup_value(query))
        return response

    def _lookup_value(self, query: np.ndarray) -> float:
        key = self._make_key(query)
        value = self.lookup_table.get(key)
        if value is not None:
            return value

        deltas = self.points - query[: self.dimension]
        distances = np.linalg.norm(deltas, axis=1)
        nearest_index = int(np.argmin(distances))
        return float(self.values[nearest_index])

    @staticmethod
    def _make_key(coords: np.ndarray) -> Tuple[float, ...]:
        flat = np.asarray(coords, dtype=np.float64).flatten()
        return tuple(round(float(value), 6) for value in flat)

    def _to_internal_coordinates(self, coords: np.ndarray) -> np.ndarray:
        if not self.latlon_enabled:
            return coords

        lat_rad = math.radians(float(coords[0]))
        lon_rad = math.radians(float(coords[1]))
        lat0 = self.latlon_meta['origin_lat_rad']
        lon0 = self.latlon_meta['origin_lon_rad']
        avg_lat = 0.5 * (lat_rad + lat0)
        delta_lat = lat_rad - lat0
        delta_lon = lon_rad - lon0
        scale = self.latlon_meta['scale']
        north = EARTH_RADIUS_M * delta_lat * scale
        east = EARTH_RADIUS_M * math.cos(avg_lat) * delta_lon * scale
        converted = np.array([north, east], dtype=np.float64)

        if coords.shape[0] > 2:
            tail = coords[2:].astype(np.float64)
            return np.concatenate((converted, tail))
        return converted


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsvSensorFieldPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
