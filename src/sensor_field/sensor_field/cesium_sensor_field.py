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
        """Load CSV and apply log10 normalization to values.

        Zero values are mapped to 0.0, non-zero values are mapped to 0.1-1.0
        to clearly distinguish between zero and non-zero measurements.
        """
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

        original_min = float(np.min(values))
        original_max = float(np.max(values))

        # Identify non-zero values
        nonzero_mask = values > 0
        nonzero_count = int(np.sum(nonzero_mask))
        zero_count = len(values) - nonzero_count

        # Initialize output array (zeros remain 0.0)
        normalized_values = np.zeros_like(values)

        if nonzero_count > 0:
            # Calculate log range from non-zero values only
            nonzero_values = values[nonzero_mask]
            log_min = float(np.log10(np.min(nonzero_values)))
            log_max = float(np.max(np.log10(nonzero_values)))

            # Update stored log_max for reference
            self._log_max = log_max

            # Map non-zero values to 0.1-1.0 range
            log_vals = np.log10(nonzero_values)
            if log_max > log_min:
                norm_vals = (log_vals - log_min) / (log_max - log_min)
            else:
                norm_vals = np.ones_like(log_vals)
            norm_vals = np.clip(norm_vals, 0.0, 1.0)
            normalized_values[nonzero_mask] = 0.1 + 0.9 * norm_vals

            self.get_logger().info(
                f'Applied log10 normalization: '
                f'original [{original_min:.0f}, {original_max:.0f}] kBq/m² -> '
                f'log [{log_min:.2f}, {log_max:.2f}] -> '
                f'zero({zero_count})=0.0, non-zero({nonzero_count})=[0.1, 1.0]'
            )
        else:
            self.get_logger().warn('All values are zero, no normalization applied')

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
