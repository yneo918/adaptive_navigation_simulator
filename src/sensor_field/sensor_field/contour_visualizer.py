#!/usr/bin/env python3
"""
ROS 2 Node for visualizing sensor field data as contour lines in RViz.

Subscribes to:
- sensor_field/points (PointCloud2): Sensor field data

Publishes:
- contour_markers (MarkerArray): Contour lines as LINE_STRIP markers
"""

import numpy as np
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from skimage import measure
from scipy.interpolate import griddata
import matplotlib.cm as cm
import matplotlib.colors as mcolors


class ContourVisualizer(Node):
    """Visualize sensor field as contour lines in RViz."""

    def __init__(self) -> None:
        super().__init__('contour_visualizer')

        # Parameters
        self.declare_parameter('contour_levels', 20)
        self.declare_parameter('show_labels', True)
        self.declare_parameter('line_width', 0.05)
        self.declare_parameter('z_offset', 0.1)
        self.declare_parameter('colormap', 'YlOrRd')
        self.declare_parameter('min_contour_points', 5)  # Minimum points to draw a contour
        self.declare_parameter('grid_resolution', 200)
        self.declare_parameter('terrain_topics', ['sensor_field/points'])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('label_interval', 2)  # Show label every N contours

        # Get parameters
        self.contour_levels = self.get_parameter('contour_levels').value
        self.show_labels = self.get_parameter('show_labels').value
        self.line_width = self.get_parameter('line_width').value
        self.z_offset = self.get_parameter('z_offset').value
        self.colormap_name = self.get_parameter('colormap').value
        self.min_contour_points = self.get_parameter('min_contour_points').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.terrain_topics = list(self.get_parameter('terrain_topics').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.label_interval = self.get_parameter('label_interval').value

        # Colormap
        self.cmap = cm.get_cmap(self.colormap_name)

        # Data storage
        self.terrain_points: np.ndarray = None
        self.terrain_elevations: np.ndarray = None
        self.terrain_received = False
        self.terrain_points_list: List[np.ndarray] = []
        self.terrain_elevations_list: List[np.ndarray] = []
        self.received_terrain_topics = set()

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'contour_markers', 10)

        # Subscribers
        self.terrain_subs = []
        for topic in self.terrain_topics:
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self._terrain_callback(msg, t),
                10
            )
            self.terrain_subs.append(sub)
            self.get_logger().info(f'Subscribed to terrain topic: {topic}')

        self.get_logger().info('=== ContourVisualizer Parameters ===')
        self.get_logger().info(f'  contour_levels: {self.contour_levels}')
        self.get_logger().info(f'  show_labels: {self.show_labels}')
        self.get_logger().info(f'  line_width: {self.line_width}')
        self.get_logger().info(f'  z_offset: {self.z_offset}')
        self.get_logger().info(f'  colormap: {self.colormap_name}')
        self.get_logger().info(f'  min_contour_points: {self.min_contour_points}')
        self.get_logger().info(f'  grid_resolution: {self.grid_resolution}')
        self.get_logger().info(f'  terrain_topics: {self.terrain_topics}')
        self.get_logger().info(f'  frame_id: {self.frame_id}')
        self.get_logger().info(f'  label_interval: {self.label_interval}')
        self.get_logger().info('====================================')
        self.get_logger().info('ContourVisualizer initialized.')

    def _terrain_callback(self, msg: PointCloud2, topic_name: str) -> None:
        """Parse terrain PointCloud2 message."""
        if topic_name in self.received_terrain_topics:
            return  # Already received from this topic

        point_step = msg.point_step
        num_points = msg.width * msg.height

        # Convert bytes to numpy array
        data = np.frombuffer(bytes(msg.data), dtype=np.float32)
        data = data.reshape(num_points, point_step // 4)

        # Extract x, y and intensity as elevation
        points = data[:, :2].copy()  # x, y
        elevations = data[:, 3].copy()  # intensity

        self.terrain_points_list.append(points)
        self.terrain_elevations_list.append(elevations)
        self.received_terrain_topics.add(topic_name)

        self.get_logger().info(
            f'Received terrain from {topic_name}: {num_points} points '
            f'({len(self.received_terrain_topics)}/{len(self.terrain_topics)})'
        )

        # Merge when all topics received
        if len(self.received_terrain_topics) >= len(self.terrain_topics):
            self._merge_and_visualize()

    def _merge_and_visualize(self) -> None:
        """Merge terrain data and generate contours."""
        self.terrain_points = np.vstack(self.terrain_points_list)
        self.terrain_elevations = np.concatenate(self.terrain_elevations_list)
        self.terrain_received = True
        self.get_logger().info(f'Terrain merged: {len(self.terrain_points)} total points')

        # Generate and publish contours
        self._generate_contours()

        # Unsubscribe from terrain topics
        for sub in self.terrain_subs:
            self.destroy_subscription(sub)
        self.terrain_subs.clear()
        self.get_logger().info('Unsubscribed from terrain topics')

        # Free memory
        self.terrain_points_list.clear()
        self.terrain_elevations_list.clear()

    def _generate_contours(self) -> None:
        """Generate contour lines from terrain data."""
        self.get_logger().info('Generating contours...')

        # Create regular grid
        x_unique = np.unique(self.terrain_points[:, 0])
        y_unique = np.unique(self.terrain_points[:, 1])

        x_min, x_max = x_unique.min(), x_unique.max()
        y_min, y_max = y_unique.min(), y_unique.max()

        grid_resolution = min(len(x_unique), len(y_unique), self.grid_resolution)
        xi = np.linspace(x_min, x_max, grid_resolution)
        yi = np.linspace(y_min, y_max, grid_resolution)
        X, Y = np.meshgrid(xi, yi)

        # Interpolate elevation data onto grid
        Z = griddata(
            self.terrain_points,
            self.terrain_elevations,
            (X, Y),
            method='linear',
            fill_value=np.nan
        )

        # Handle NaN values
        mask = ~np.isnan(Z)
        if not mask.any():
            self.get_logger().warn('No valid elevation data for contouring')
            return

        # Determine contour levels
        z_min, z_max = np.nanmin(Z), np.nanmax(Z)
        levels = np.linspace(z_min, z_max, self.contour_levels + 1)

        self.get_logger().info(f'Elevation range: [{z_min:.3f}, {z_max:.3f}]')
        self.get_logger().info(f'Generating {len(levels)} contour levels')

        # Create MarkerArray
        marker_array = MarkerArray()
        marker_id = 0

        # Normalize colormap
        norm = mcolors.Normalize(vmin=z_min, vmax=z_max)

        # Generate contours for each level
        for level_idx, level in enumerate(levels):
            try:
                # Find contours at this level
                contours = measure.find_contours(Z, level)

                for contour in contours:
                    if len(contour) < self.min_contour_points:
                        continue

                    # Convert grid coordinates to world coordinates
                    contour_x = np.interp(contour[:, 1], np.arange(len(xi)), xi)
                    contour_y = np.interp(contour[:, 0], np.arange(len(yi)), yi)

                    # Create LINE_STRIP marker
                    marker = self._create_line_marker(
                        marker_id, contour_x, contour_y, level, norm
                    )
                    marker_array.markers.append(marker)
                    marker_id += 1

                    # Add label if enabled and at the right interval
                    if self.show_labels and level_idx % self.label_interval == 0:
                        # Place label at the middle of the contour
                        mid_idx = len(contour_x) // 2
                        label_marker = self._create_text_marker(
                            marker_id, contour_x[mid_idx], contour_y[mid_idx],
                            level, norm
                        )
                        marker_array.markers.append(label_marker)
                        marker_id += 1

            except Exception as e:
                self.get_logger().warn(f'Error generating contour at level {level}: {e}')
                continue

        self.get_logger().info(f'Generated {len(marker_array.markers)} markers')
        self.marker_pub.publish(marker_array)
        self.get_logger().info('Contours published to /contour_markers')

    def _create_line_marker(
        self, marker_id: int, x: np.ndarray, y: np.ndarray,
        level: float, norm: mcolors.Normalize
    ) -> Marker:
        """Create a LINE_STRIP marker for a contour."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'contour_lines'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Line width
        marker.scale.x = self.line_width

        # Color from colormap
        rgba = self.cmap(norm(level))
        marker.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]),
                                 b=float(rgba[2]), a=1.0)

        # Points
        from geometry_msgs.msg import Point
        for xi, yi in zip(x, y):
            point = Point()
            point.x = float(xi)
            point.y = float(yi)
            point.z = self.z_offset
            marker.points.append(point)

        return marker

    def _create_text_marker(
        self, marker_id: int, x: float, y: float,
        level: float, norm: mcolors.Normalize
    ) -> Marker:
        """Create a TEXT_VIEW_FACING marker for a contour label."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'contour_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = self.z_offset + 0.5
        marker.pose.orientation.w = 1.0

        # Text
        marker.text = f'{level:.2f}'
        marker.scale.z = 0.5  # Text height

        # Color (black for visibility)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContourVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
