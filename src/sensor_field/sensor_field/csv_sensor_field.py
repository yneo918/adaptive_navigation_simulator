import csv
import hashlib
import json
import math
import pickle
import re
import struct
from pathlib import Path
from typing import Dict, Optional, Tuple, List

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
        self.declare_parameter('fill_pointcloud_topic', 'sensor_field/fill_points')
        self.declare_parameter('publish_interval', 10.0)
        self.declare_parameter('column_x', 'x')
        self.declare_parameter('column_y', 'y')
        self.declare_parameter('column_z', '')
        self.declare_parameter('column_data', 'a')
        self.declare_parameter('interpret_latlon', False)
        self.declare_parameter('origin_latitude', float('nan'))
        self.declare_parameter('origin_longitude', float('nan'))
        self.declare_parameter('distance_scale', 1.0)
        self.declare_parameter('fill_rect_min', '')
        self.declare_parameter('fill_rect_max', '')
        self.declare_parameter('fill_spacing', 1.0)
        self.declare_parameter('fill_radius', 0.0)
        self.declare_parameter('fill_neighbor_count', 12)
        self.declare_parameter('fill_weight_power', 2.0)
        self.declare_parameter('fill_method', 'idw')
        self.declare_parameter('fill_tps_regularization', 1e-5)
        self.declare_parameter('service_inputs_latlon', False)
        self.declare_parameter('clip_points_to_fill_rect', False)
        self.declare_parameter('cache_dir', '.sensor_field_cache')
        self.declare_parameter('enable_cache', True)

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        if not csv_path:
            raise ValueError('Parameter "csv_path" must be set to a CSV file path.')

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        fill_pointcloud_topic = (
            self.get_parameter('fill_pointcloud_topic').get_parameter_value().string_value
        )
        publish_interval = self.get_parameter('publish_interval').get_parameter_value().double_value

        column_x = self.get_parameter('column_x').get_parameter_value().string_value
        column_y = self.get_parameter('column_y').get_parameter_value().string_value
        column_z_param = self.get_parameter('column_z').get_parameter_value().string_value
        column_data = self.get_parameter('column_data').get_parameter_value().string_value

        if not column_x or not column_y or not column_data:
            raise ValueError('Parameters "column_x", "column_y", and "column_data" must be set to column names.')

        column_z = column_z_param if column_z_param else None

        interpret_latlon = self.get_parameter('interpret_latlon').get_parameter_value().bool_value
        origin_latitude = self.get_parameter('origin_latitude').get_parameter_value().double_value
        origin_longitude = self.get_parameter('origin_longitude').get_parameter_value().double_value
        service_inputs_latlon = (
            self.get_parameter('service_inputs_latlon').get_parameter_value().bool_value
        )
        distance_scale = self.get_parameter('distance_scale').get_parameter_value().double_value
        if distance_scale <= 0.0:
            raise ValueError('Parameter "distance_scale" must be a positive value.')

        fill_rect_min_param = self.get_parameter('fill_rect_min').get_parameter_value().string_value
        fill_rect_max_param = self.get_parameter('fill_rect_max').get_parameter_value().string_value
        fill_spacing = self.get_parameter('fill_spacing').get_parameter_value().double_value
        fill_radius = self.get_parameter('fill_radius').get_parameter_value().double_value
        fill_neighbor_count = (
            self.get_parameter('fill_neighbor_count').get_parameter_value().integer_value
        )
        fill_weight_power = self.get_parameter('fill_weight_power').get_parameter_value().double_value
        fill_method = self.get_parameter('fill_method').get_parameter_value().string_value or 'idw'
        fill_tps_regularization = (
            self.get_parameter('fill_tps_regularization').get_parameter_value().double_value
        )
        clip_points_to_fill_rect = (
            self.get_parameter('clip_points_to_fill_rect').get_parameter_value().bool_value
        )
        cache_dir = self.get_parameter('cache_dir').get_parameter_value().string_value
        enable_cache = self.get_parameter('enable_cache').get_parameter_value().bool_value

        if fill_spacing <= 0.0:
            raise ValueError('Parameter "fill_spacing" must be positive.')
        if fill_radius < 0.0:
            raise ValueError('Parameter "fill_radius" must be non-negative.')
        if fill_neighbor_count <= 0:
            raise ValueError('Parameter "fill_neighbor_count" must be a positive integer.')
        if fill_weight_power <= 0.0:
            raise ValueError('Parameter "fill_weight_power" must be positive.')
        if fill_method not in ('idw', 'thin_plate'):
            raise ValueError('Parameter "fill_method" must be either "idw" or "thin_plate".')
        if fill_tps_regularization < 0.0:
            raise ValueError('Parameter "fill_tps_regularization" must be non-negative.')

        fill_min = self._parse_rect_corner(fill_rect_min_param)
        fill_max = self._parse_rect_corner(fill_rect_max_param)
        if (fill_min is None) ^ (fill_max is None):
            raise ValueError('Both "fill_rect_min" and "fill_rect_max" must be provided together.')

        # Build cache configuration
        cache_config = {
            'csv_path': csv_path,
            'column_x': column_x,
            'column_y': column_y,
            'column_z': column_z,
            'column_data': column_data,
            'interpret_latlon': interpret_latlon,
            'origin_latitude': origin_latitude,
            'origin_longitude': origin_longitude,
            'distance_scale': distance_scale,
            'fill_rect_min': fill_rect_min_param,
            'fill_rect_max': fill_rect_max_param,
            'fill_spacing': fill_spacing,
            'fill_radius': fill_radius,
            'fill_neighbor_count': fill_neighbor_count,
            'fill_weight_power': fill_weight_power,
            'fill_method': fill_method,
            'fill_tps_regularization': fill_tps_regularization,
            'clip_points_to_fill_rect': clip_points_to_fill_rect,
        }

        # Try loading from cache
        cache_loaded = False
        if enable_cache:
            cached_data = self._load_from_cache(cache_dir, cache_config)
            if cached_data is not None:
                points = cached_data['points']
                values = cached_data['values']
                dimension = cached_data['dimension']
                latlon_meta = cached_data['latlon_meta']
                base_points = cached_data['base_points']
                base_values = cached_data['base_values']
                fill_points = cached_data['fill_points']
                fill_values = cached_data['fill_values']
                tps_models = cached_data.get('tps_models', [])
                tps_tail_models = cached_data.get('tps_tail_models', [])
                cache_loaded = True
                self.get_logger().info('Loaded sensor field from cache.')

        if not cache_loaded:
            points, values, dimension, latlon_meta = self._load_csv(
                Path(csv_path),
                column_x,
                column_y,
                column_z,
                column_data,
                interpret_latlon,
                origin_latitude,
                origin_longitude,
                distance_scale,
            )
        self.dimension = dimension
        self.interpolation_radius = float(fill_radius)
        self.fill_neighbor_count = int(fill_neighbor_count)
        self.fill_weight_power = float(fill_weight_power)
        self.fill_method = fill_method
        self.fill_tps_regularization = float(fill_tps_regularization)

        if not cache_loaded:
            self.source_points = points.copy()
            self.source_values = values.copy()
            self._tps_models: list[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
            self._tps_tail_models: list[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
            self._tps_ready = False

            if self.fill_method == 'thin_plate':
                self._prepare_tps_models(self.source_points, self.source_values)

            base_points, base_values, fill_points, fill_values = self._augment_with_grid(
                self.source_points,
                self.source_values,
                self.dimension,
                fill_min,
                fill_max,
                float(fill_spacing),
                float(fill_radius),
            )

            # Save to cache
            if enable_cache:
                cache_data = {
                    'points': points,
                    'values': values,
                    'dimension': dimension,
                    'latlon_meta': latlon_meta,
                    'base_points': base_points,
                    'base_values': base_values,
                    'fill_points': fill_points,
                    'fill_values': fill_values,
                    'tps_models': self._tps_models,
                    'tps_tail_models': self._tps_tail_models,
                }
                self._save_to_cache(cache_dir, cache_config, cache_data)
        else:
            self.source_points = points.copy()
            self.source_values = values.copy()
            self._tps_models = tps_models
            self._tps_tail_models = tps_tail_models
            self._tps_ready = len(tps_models) > 0
        self.frame_id = frame_id
        self.points = base_points
        self.values = base_values
        self.fill_points = fill_points
        self.fill_values = fill_values
        self.grid_points_added = int(self.fill_points.shape[0])
        self.latlon_meta = latlon_meta
        self.latlon_enabled = latlon_meta is not None
        self.service_inputs_latlon = service_inputs_latlon and self.latlon_enabled
        all_points = self.points if self.fill_points.size == 0 else np.vstack((self.points, self.fill_points))
        all_values = self.values if self.fill_values.size == 0 else np.concatenate((self.values, self.fill_values))
        self.lookup_table: Dict[Tuple[float, ...], float] = {
            self._make_key(point): float(value)
            for point, value in zip(all_points, all_values)
        }

        # Clip base points to fill_rect if enabled
        publish_points = self.points
        publish_values = self.values
        clipped_count = 0
        if clip_points_to_fill_rect and fill_min is not None and fill_max is not None:
            publish_points, publish_values, clipped_count = self._clip_points_to_rect(
                self.points, self.values, fill_min, fill_max
            )

        self.point_step = struct.calcsize('ffff')
        self.base_pointcloud_msg = self._create_pointcloud_message(publish_points, publish_values)
        self.fill_pointcloud_msg = (
            self._create_pointcloud_message(self.fill_points, self.fill_values)
            if self.fill_points.size > 0
            else None
        )

        self.publisher = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self.fill_publisher = None
        if fill_pointcloud_topic and self.fill_points.size > 0:
            self.fill_publisher = self.create_publisher(PointCloud2, fill_pointcloud_topic, 10)

        self.column_names = {
            'x': column_x,
            'y': column_y,
            'z': column_z,
            'data': column_data,
        }

        if self.dimension == 2:
            self.service = self.create_service(GetSensor2D, 'get_sensor', self._handle_request_2d)
            message = (
                f'Loaded 2D sensor field from {csv_path} with {len(self.points)} samples '
                f'(columns x={column_x}, y={column_y}, data={column_data})'
            )
        else:
            self.service = self.create_service(GetSensor3D, 'get_sensor', self._handle_request_3d)
            message = (
                f'Loaded 3D sensor field from {csv_path} with {len(self.points)} samples '
                f'(columns x={column_x}, y={column_y}, z={column_z}, data={column_data})'
            )

        if self.grid_points_added:
            message += f' (+{self.grid_points_added} interpolated grid points)'

        if clipped_count > 0:
            message += f' ({clipped_count} points clipped to fill_rect)'

        self.get_logger().info(message + '.')

        if self.latlon_enabled:
            origin_lat = math.degrees(self.latlon_meta['origin_lat_rad'])
            origin_lon = math.degrees(self.latlon_meta['origin_lon_rad'])
            self.get_logger().info(
                f'Interpreting x/y as latitude/longitude (origin {origin_lat:.6f}°, '
                f'{origin_lon:.6f}°, scale={self.latlon_meta["scale"]:.6f}).'
            )
        self._publish_pointclouds()
        self.timer = self.create_timer(max(publish_interval, 0.01), self._publish_pointclouds)

    def _load_csv(
        self,
        path: Path,
        column_x: str,
        column_y: str,
        column_z: Optional[str],
        column_data: str,
        interpret_latlon: bool,
        origin_latitude: float,
        origin_longitude: float,
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
            # Use specified origin if provided, otherwise use first data point
            if not math.isnan(origin_latitude) and not math.isnan(origin_longitude):
                lat0 = math.radians(origin_latitude)
                lon0 = math.radians(origin_longitude)
            else:
                lat0 = float(lat_rad[0])
                lon0 = float(lon_rad[0])
            scale = float(distance_scale)
            avg_lat = 0.5 * (lat_rad + lat0)
            delta_lat = lat_rad - lat0
            delta_lon = lon_rad - lon0
            east = EARTH_RADIUS_M * np.cos(avg_lat) * delta_lon * scale
            north = EARTH_RADIUS_M * delta_lat * scale
            converted_xy = np.column_stack((east, north))
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

    def _create_pointcloud_message(
        self,
        points: np.ndarray,
        values: np.ndarray,
        frame_id: Optional[str] = None,
    ) -> PointCloud2:
        msg = PointCloud2()
        msg.header.frame_id = frame_id if frame_id is not None else self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = self.point_step
        msg.row_step = self.point_step * len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        if points.size == 0:
            msg.data = bytes()
            return msg

        if self.dimension == 2:
            padded_points = np.column_stack((points, np.zeros(len(points))))
        else:
            padded_points = points

        intensity = values.astype(np.float32, copy=False)
        buffer = bytearray(self.point_step * len(padded_points))
        for index, (x, y, z) in enumerate(padded_points):
            struct.pack_into('ffff', buffer, index * self.point_step, float(x), float(y), intensity[index], intensity[index])

        msg.data = bytes(buffer)
        return msg

    def _publish_pointclouds(self) -> None:
        now = self.get_clock().now().to_msg()

        self.base_pointcloud_msg.header.stamp = now
        self.publisher.publish(self.base_pointcloud_msg)

        if self.fill_pointcloud_msg and self.fill_publisher:
            self.fill_pointcloud_msg.header.stamp = now
            self.fill_publisher.publish(self.fill_pointcloud_msg)

    def _handle_request_2d(self, request: GetSensor2D.Request, response: GetSensor2D.Response) -> GetSensor2D.Response:
        query = np.array([request.x, request.y], dtype=np.float64)
        response.data = float(self._lookup_value(query))
        return response

    def _handle_request_3d(self, request: GetSensor3D.Request, response: GetSensor3D.Response) -> GetSensor3D.Response:
        query = np.array([request.x, request.y, request.z], dtype=np.float64)
        response.data = float(self._lookup_value(query))
        return response

    def _lookup_value(self, query: np.ndarray) -> float:
        internal_query = self._ensure_internal_coordinates(query[: self.dimension])
        key = self._make_key(internal_query)
        value = self.lookup_table.get(key)
        if value is not None:
            return value

        interpolated, _, _, _, _ = self._interpolate_from_samples(
            internal_query,
            self.points,
            self.values,
            self.interpolation_radius,
        )
        return interpolated

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
        east = EARTH_RADIUS_M * math.cos(avg_lat) * delta_lon * scale
        north = EARTH_RADIUS_M * delta_lat * scale
        converted = np.array([east, north], dtype=np.float64)

        if coords.shape[0] > 2:
            tail = coords[2:].astype(np.float64)
            return np.concatenate((converted, tail))
        return converted

    def _ensure_internal_coordinates(self, coords: np.ndarray) -> np.ndarray:
        if self.service_inputs_latlon:
            return self._to_internal_coordinates(coords)
        return coords

    def _compute_cache_hash(self, cache_config: Dict) -> str:
        """Compute hash for cache configuration including CSV file content."""
        hasher = hashlib.sha256()

        # Hash CSV file content
        csv_path = Path(cache_config['csv_path'])
        if csv_path.exists():
            with csv_path.open('rb') as f:
                hasher.update(f.read())

        # Hash configuration parameters (sorted for consistency)
        config_str = json.dumps(cache_config, sort_keys=True)
        hasher.update(config_str.encode('utf-8'))

        return hasher.hexdigest()

    def _get_cache_path(self, cache_dir: str, cache_hash: str) -> Path:
        """Get cache file path."""
        cache_path = Path(cache_dir)
        cache_path.mkdir(parents=True, exist_ok=True)
        return cache_path / f'{cache_hash}.pkl'

    def _load_from_cache(self, cache_dir: str, cache_config: Dict) -> Optional[Dict]:
        """Load cached data if available and valid."""
        try:
            cache_hash = self._compute_cache_hash(cache_config)
            cache_file = self._get_cache_path(cache_dir, cache_hash)

            if not cache_file.exists():
                return None

            with cache_file.open('rb') as f:
                cached = pickle.load(f)

            # Verify cache hash matches
            if cached.get('cache_hash') != cache_hash:
                self.get_logger().warning('Cache hash mismatch, regenerating...')
                return None

            return cached.get('data')
        except Exception as e:
            self.get_logger().warning(f'Failed to load cache: {e}')
            return None

    def _save_to_cache(self, cache_dir: str, cache_config: Dict, data: Dict) -> None:
        """Save data to cache file."""
        try:
            cache_hash = self._compute_cache_hash(cache_config)
            cache_file = self._get_cache_path(cache_dir, cache_hash)

            cache_payload = {
                'cache_hash': cache_hash,
                'config': cache_config,
                'data': data,
            }

            with cache_file.open('wb') as f:
                pickle.dump(cache_payload, f, protocol=pickle.HIGHEST_PROTOCOL)

            self.get_logger().info(f'Saved sensor field to cache: {cache_file}')
        except Exception as e:
            self.get_logger().warning(f'Failed to save cache: {e}')

    @staticmethod
    def _parse_rect_corner(raw: str) -> Optional[Tuple[float, float]]:
        text = raw.strip()
        if not text:
            return None

        parts = [part for part in re.split(r'[\s,]+', text) if part]
        if len(parts) != 2:
            raise ValueError(
                'Rectangle corners must contain two numeric components separated by space or comma.'
            )
        try:
            return float(parts[0]), float(parts[1])
        except ValueError as exc:
            raise ValueError('Rectangle corner values must be numeric.') from exc

    @staticmethod
    def _clip_points_to_rect(
        points: np.ndarray,
        values: np.ndarray,
        rect_min: Tuple[float, float],
        rect_max: Tuple[float, float],
    ) -> Tuple[np.ndarray, np.ndarray, int]:
        """Clip points to within the specified rectangle bounds.

        Returns:
            Tuple of (clipped_points, clipped_values, removed_count)
        """
        if points.size == 0:
            return points, values, 0

        min_x, min_y = min(rect_min[0], rect_max[0]), min(rect_min[1], rect_max[1])
        max_x, max_y = max(rect_min[0], rect_max[0]), max(rect_min[1], rect_max[1])

        # Check which points are within bounds (using first 2 dimensions)
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        mask = (x_coords >= min_x) & (x_coords <= max_x) & (y_coords >= min_y) & (y_coords <= max_y)

        original_count = len(points)
        clipped_points = points[mask]
        clipped_values = values[mask]
        removed_count = original_count - len(clipped_points)

        return clipped_points, clipped_values, removed_count

    def _prepare_tps_models(self, sample_points: np.ndarray, sample_values: np.ndarray) -> None:
        if sample_points.shape[1] < 2:
            self.get_logger().warning(
                'fill_method "thin_plate" requires at least two spatial dimensions; falling back to IDW.'
            )
            self.fill_method = 'idw'
            return

        sample_count = sample_points.shape[0]
        if sample_count < 3:
            self.get_logger().warning(
                'fill_method "thin_plate" requires at least three samples; falling back to IDW.'
            )
            self.fill_method = 'idw'
            return

        xy = sample_points[:, :2]

        try:
            main_model = self._build_tps_model(xy, sample_values)
        except np.linalg.LinAlgError as exc:
            self.get_logger().warning(
                'Thin-plate spline preparation failed (%s); falling back to IDW.', str(exc)
            )
            self.fill_method = 'idw'
            return

        tail_models: list[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        if self.dimension > 2:
            for axis in range(2, self.dimension):
                axis_values = sample_points[:, axis]
                try:
                    tail_models.append(self._build_tps_model(xy, axis_values))
                except np.linalg.LinAlgError as exc:
                    self.get_logger().warning(
                        'Thin-plate spline for extra dimension failed (%s); falling back to IDW.',
                        str(exc),
                    )
                    self.fill_method = 'idw'
                    return

        self._tps_models = [main_model]
        self._tps_tail_models = tail_models
        self._tps_ready = True

    def _build_tps_model(
        self,
        xy: np.ndarray,
        values: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        n_samples = xy.shape[0]
        diff = xy[:, None, :] - xy[None, :, :]
        r = np.linalg.norm(diff, axis=2)
        kernel = self._tps_kernel(r)
        if self.fill_tps_regularization > 0.0:
            kernel += self.fill_tps_regularization * np.eye(n_samples, dtype=np.float64)

        p = np.column_stack((np.ones(n_samples), xy))

        system = np.zeros((n_samples + 3, n_samples + 3), dtype=np.float64)
        system[:n_samples, :n_samples] = kernel
        system[:n_samples, n_samples:] = p
        system[n_samples:, :n_samples] = p.T

        rhs = np.zeros(n_samples + 3, dtype=np.float64)
        rhs[:n_samples] = values

        try:
            solution = np.linalg.solve(system, rhs)
        except np.linalg.LinAlgError:
            solution, *_ = np.linalg.lstsq(system, rhs, rcond=None)

        weights = solution[:n_samples]
        affine = solution[n_samples:]
        return weights.astype(np.float64), affine.astype(np.float64), xy.astype(np.float64)

    @staticmethod
    def _tps_kernel(r: np.ndarray) -> np.ndarray:
        with np.errstate(divide='ignore', invalid='ignore'):
            result = r * r * np.where(r > 0.0, np.log(r), 0.0)
        result[np.isnan(result)] = 0.0
        return result

    @staticmethod
    def _evaluate_tps_model(
        model: Tuple[np.ndarray, np.ndarray, np.ndarray],
        query_xy: np.ndarray,
    ) -> float:
        weights, affine, control_points = model
        distances = np.linalg.norm(control_points - query_xy, axis=1)
        kernel = CsvSensorFieldPublisher._tps_kernel(distances)
        value = float(np.dot(weights, kernel) + affine[0] + affine[1] * query_xy[0] + affine[2] * query_xy[1])
        return value

    def _augment_with_grid(
        self,
        source_points: np.ndarray,
        source_values: np.ndarray,
        dimension: int,
        rect_min: Optional[Tuple[float, float]],
        rect_max: Optional[Tuple[float, float]],
        spacing: float,
        radius: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if rect_min is None or rect_max is None:
            return source_points.copy(), source_values.copy(), 0

        if dimension < 2:
            raise ValueError('Grid interpolation requires at least 2D sensor data.')

        x0, y0 = rect_min
        x1, y1 = rect_max
        min_x, max_x = sorted((x0, x1))
        min_y, max_y = sorted((y0, y1))

        if math.isclose(max_x, min_x) or math.isclose(max_y, min_y):
            return source_points.copy(), source_values.copy(), 0

        x_coords = np.arange(min_x, max_x + spacing * 0.5, spacing, dtype=np.float64)
        y_coords = np.arange(min_y, max_y + spacing * 0.5, spacing, dtype=np.float64)

        existing_keys = {self._make_key(point) for point in source_points}
        fill_points: List[np.ndarray] = []
        fill_values: List[float] = []

        for x in x_coords:
            for y in y_coords:
                candidate_xy = np.array([x, y], dtype=np.float64)

                try:
                    (
                        interpolated_value,
                        neighbor_indices,
                        weights,
                        _,
                        tail_values,
                    ) = (
                        self._interpolate_from_samples(
                            candidate_xy,
                            source_points,
                            source_values,
                            radius,
                        )
                    )
                except ValueError:
                    continue

                if dimension > 2:
                    if tail_values is not None:
                        candidate_tail = np.atleast_1d(tail_values).astype(np.float64)
                    else:
                        neighbor_coords = source_points[neighbor_indices, 2:]
                        averaged_tail = np.average(neighbor_coords, axis=0, weights=weights)
                        candidate_tail = np.atleast_1d(averaged_tail).astype(np.float64)
                    candidate = np.concatenate((candidate_xy, candidate_tail))
                else:
                    candidate = candidate_xy

                key = self._make_key(candidate)
                if key in existing_keys:
                    continue

                existing_keys.add(key)
                fill_points.append(candidate.astype(np.float64))
                fill_values.append(interpolated_value)

        base_points = source_points.copy()
        base_values = source_values.copy()

        if not fill_points:
            empty_points = np.empty((0, dimension), dtype=np.float64)
            empty_values = np.empty((0,), dtype=np.float64)
            return base_points, base_values, empty_points, empty_values

        fill_points_array = np.vstack(fill_points)
        fill_values_array = np.asarray(fill_values, dtype=np.float64)

        return base_points, base_values, fill_points_array, fill_values_array

    def _interpolate_from_samples(
        self,
        query: np.ndarray,
        sample_points: np.ndarray,
        sample_values: np.ndarray,
        radius: float,
    ) -> Tuple[float, np.ndarray, np.ndarray, float, Optional[np.ndarray]]:
        if sample_points.size == 0:
            raise ValueError('Sensor field is empty; cannot interpolate values.')

        dims = min(2, sample_points.shape[1])
        query_slice = query[:dims]
        dataset_slice = sample_points[:, :dims]

        if self.fill_method == 'thin_plate' and self._tps_ready:
            main_value = self._evaluate_tps_model(self._tps_models[0], query_slice)
            tail_values = None
            if self.dimension > 2 and self._tps_tail_models:
                tail_values = np.asarray(
                    [self._evaluate_tps_model(model, query_slice) for model in self._tps_tail_models],
                    dtype=np.float64,
                )
            return main_value, np.array([], dtype=int), np.array([], dtype=np.float64), 0.0, tail_values

        distances = np.linalg.norm(dataset_slice - query_slice, axis=1)
        if distances.size == 0:
            raise ValueError('No sample distances available for interpolation.')

        zero_mask = distances <= 1e-6
        if np.any(zero_mask):
            first_index = int(np.where(zero_mask)[0][0])
            tail_values = None
            if self.dimension > 2:
                tail_values = sample_points[first_index, 2:].astype(np.float64)
            return (
                float(sample_values[first_index]),
                np.array([first_index]),
                np.array([1.0], dtype=np.float64),
                0.0,
                tail_values,
            )

        candidate_indices = np.arange(distances.size)
        if radius > 0.0:
            within_radius = np.where(distances <= radius)[0]
            if within_radius.size >= self.fill_neighbor_count:
                candidate_indices = within_radius

        candidate_distances = distances[candidate_indices]
        k = min(self.fill_neighbor_count, candidate_indices.size)
        if k == 0:
            raise ValueError('Not enough neighbours available for interpolation.')

        selected = np.argpartition(candidate_distances, k - 1)[:k]
        neighbor_indices = candidate_indices[selected]
        neighbor_distances = distances[neighbor_indices]

        weights = 1.0 / np.power(np.maximum(neighbor_distances, 1e-6), self.fill_weight_power)
        weight_sum = float(np.sum(weights))
        if weight_sum == 0.0:
            weights = np.full(weights.shape, 1.0 / weights.size, dtype=np.float64)
        else:
            weights = weights / weight_sum

        interpolated = float(np.dot(sample_values[neighbor_indices], weights))
        min_distance = float(np.min(neighbor_distances))

        tail_values = None
        if self.dimension > 2:
            neighbor_coords = sample_points[neighbor_indices, 2:]
            tail_values = np.average(neighbor_coords, axis=0, weights=weights)

        return (
            interpolated,
            neighbor_indices,
            weights.astype(np.float64),
            min_distance,
            None if tail_values is None else np.asarray(tail_values, dtype=np.float64),
        )


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
