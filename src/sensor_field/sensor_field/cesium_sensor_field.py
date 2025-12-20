"""Cesium concentration sensor field with log10 normalization.

This module extends CsvSensorFieldPublisher to apply log10(x+1) normalization
to cesium concentration data (kBq/m²), mapping values to 0.0-1.0 range.
"""

import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy

from sensor_field.csv_sensor_field import CsvSensorFieldPublisher


class CesiumSensorFieldPublisher(CsvSensorFieldPublisher):
    """Sensor field publisher with log10 normalization for cesium concentration data."""

    def __init__(self) -> None:
        # Declare additional parameters before calling parent __init__
        # We need to intercept and modify values after CSV loading
        self._log_max: float = 7.03  # Default: log10(10,736,625 + 1)

        super().__init__()

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
        """Load CSV and apply log10(x+1) normalization to values."""
        points, values, dimension, latlon_meta = super()._load_csv(
            path,
            column_x,
            column_y,
            column_z,
            column_data,
            interpret_latlon,
            origin_latitude,
            origin_longitude,
            distance_scale,
        )

        # Apply log10(x+1) normalization
        original_min = float(np.min(values))
        original_max = float(np.max(values))

        # log10(x+1) transformation
        log_values = np.log10(values + 1.0)
        log_max = float(np.max(log_values))

        # Update stored log_max for reference
        self._log_max = log_max if log_max > 0 else 1.0

        # Normalize to 0.0-1.0
        if log_max > 0:
            normalized_values = log_values / log_max
        else:
            normalized_values = log_values

        self.get_logger().info(
            f'Applied log10(x+1) normalization: '
            f'original range [{original_min:.0f}, {original_max:.0f}] kBq/m² -> '
            f'log range [0, {log_max:.3f}] -> normalized [0.0, 1.0]'
        )

        return points, normalized_values, dimension, latlon_meta


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CesiumSensorFieldPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
