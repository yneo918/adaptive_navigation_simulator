"""
ROS 2 Node for 3D terrain visualization with rover trajectories.

Subscribes to:
- sensor_field/points (PointCloud2): Terrain elevation data
- /sim/p{i}/pose2D (Pose2D): Rover positions

Creates a matplotlib 3D plot showing:
- Terrain as a surface
- Rover trajectories as lines and points on the terrain surface
"""

import os
import struct
import sys
from collections import defaultdict
from datetime import datetime
from typing import Dict, List, Set, Tuple

# Fix mpl_toolkits namespace package conflict between system and user installations
# Must be done before importing matplotlib/mpl_toolkits
_user_site = '/home/neo/.local/lib/python3.10/site-packages'
sys.path.insert(0, _user_site)
# Clear any cached matplotlib/mpl_toolkits imports to ensure user packages take precedence
for _mod in [k for k in list(sys.modules.keys()) if 'mpl_toolkits' in k or 'matplotlib' in k]:
    del sys.modules[_mod]

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers '3d' projection
from scipy.interpolate import griddata, LinearNDInterpolator, NearestNDInterpolator

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import PointCloud2

from robot_interfaces.srv import GetSensor2D


class TrajectoryPlotter3D(Node):
    """3D terrain and trajectory visualization node."""

    def __init__(self) -> None:
        super().__init__('trajectory_plotter_3d')

        # Parameters
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('robot_prefix', '')
        self.declare_parameter('robot_ids', ['p1', 'p2', 'p3', 'p4', 'p5'])
        self.declare_parameter('trajectory_sample_interval', 0.5)  # seconds
        self.declare_parameter('output_dir', 'trajectory')  # Output directory for all files
        self.declare_parameter('output_file', 'trajectory_3d.png')  # Base filename (timestamp will be prepended)
        self.declare_parameter('dpi', 150)
        self.declare_parameter('elevation_scale', 1.0)  # Scale factor for Z axis
        self.declare_parameter('z_aspect_ratio', 0.2)  # Z axis display ratio (X:Y:Z = 1:1:z_aspect_ratio)
        self.declare_parameter('terrain_alpha', 0.7)
        self.declare_parameter('line_width', 2.0)
        self.declare_parameter('marker_size', 10)
        self.declare_parameter('marker_interval', 10)  # Plot marker every N points
        self.declare_parameter('trajectory_z_offset', 1.0)  # Z offset to lift trajectory above terrain
        self.declare_parameter('terrain_topics', ['sensor_field/points'])
        self.declare_parameter('show_lines', True)  # Show trajectory lines
        self.declare_parameter('line_robot_ids', ['p1', 'p2', 'p3', 'p4', 'p5'])  # Robot IDs to show lines (empty = all if show_lines=True)

        # Get parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.robot_ids = list(self.get_parameter('robot_ids').value)
        self.sample_interval = self.get_parameter('trajectory_sample_interval').value
        self.output_dir = self.get_parameter('output_dir').value
        self.output_file_base = self.get_parameter('output_file').value
        self.dpi = self.get_parameter('dpi').value

        # Generate timestamp for filenames
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        self.elevation_scale = self.get_parameter('elevation_scale').value
        self.z_aspect_ratio = self.get_parameter('z_aspect_ratio').value
        self.terrain_alpha = self.get_parameter('terrain_alpha').value
        self.line_width = self.get_parameter('line_width').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_interval = self.get_parameter('marker_interval').value
        self.trajectory_z_offset = self.get_parameter('trajectory_z_offset').value
        self.terrain_topics: List[str] = list(self.get_parameter('terrain_topics').value)
        self.show_lines = self.get_parameter('show_lines').value
        self.line_robot_ids: List[str] = list(self.get_parameter('line_robot_ids').value)

        # Terrain data storage
        self.terrain_points: np.ndarray = None
        self.terrain_elevations: np.ndarray = None
        self.terrain_received = False
        self.terrain_points_list: List[np.ndarray] = []
        self.terrain_elevations_list: List[np.ndarray] = []
        self.received_terrain_topics: Set[str] = set()

        # Interpolators (cached for performance)
        self._linear_interp = None
        self._nearest_interp = None

        # Trajectory storage: robot_id -> list of (x, y, timestamp)
        self.trajectories: Dict[str, List[Tuple[float, float, float]]] = defaultdict(list)
        self.last_sample_time: Dict[str, float] = {}

        # Colors for different robots
        self.robot_colors = ['red', 'blue', 'green', 'orange', 'purple',
                             'cyan', 'magenta', 'yellow', 'brown', 'pink']

        # Subscribe to terrain data (multiple topics)
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

        # Subscribe to rover positions
        self.pose_subs = []
        for robot_id in self.robot_ids[:self.num_robots]:
            topic = f'{self.robot_prefix}/{robot_id}/pose2D'
            sub = self.create_subscription(
                Pose2D,
                topic,
                lambda msg, rid=robot_id: self._pose_callback(msg, rid),
                10
            )
            self.pose_subs.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

        # Service client for elevation queries
        self.elevation_client = self.create_client(GetSensor2D, 'get_sensor')

        # Log all parameters
        self.get_logger().info('=== TrajectoryPlotter3D Parameters ===')
        self.get_logger().info(f'  num_robots: {self.num_robots}')
        self.get_logger().info(f'  robot_prefix: "{self.robot_prefix}"')
        self.get_logger().info(f'  robot_ids: {self.robot_ids}')
        self.get_logger().info(f'  trajectory_sample_interval: {self.sample_interval}')
        self.get_logger().info(f'  output_dir: {self.output_dir}')
        self.get_logger().info(f'  output_file: {self.output_file_base}')
        self.get_logger().info(f'  timestamp: {self.timestamp}')
        self.get_logger().info(f'  dpi: {self.dpi}')
        self.get_logger().info(f'  elevation_scale: {self.elevation_scale}')
        self.get_logger().info(f'  z_aspect_ratio: {self.z_aspect_ratio}')
        self.get_logger().info(f'  terrain_alpha: {self.terrain_alpha}')
        self.get_logger().info(f'  line_width: {self.line_width}')
        self.get_logger().info(f'  marker_size: {self.marker_size}')
        self.get_logger().info(f'  marker_interval: {self.marker_interval}')
        self.get_logger().info(f'  trajectory_z_offset: {self.trajectory_z_offset}')
        self.get_logger().info(f'  terrain_topics: {self.terrain_topics}')
        self.get_logger().info(f'  show_lines: {self.show_lines}')
        self.get_logger().info(f'  line_robot_ids: {self.line_robot_ids}')
        self.get_logger().info('======================================')
        self.get_logger().info('TrajectoryPlotter3D initialized. Press Ctrl+C to generate plot.')

    def _terrain_callback(self, msg: PointCloud2, topic_name: str) -> None:
        """Parse terrain PointCloud2 message from multiple topics."""
        if topic_name in self.received_terrain_topics:
            return  # Already received from this topic

        point_step = msg.point_step
        num_points = msg.width * msg.height

        # Convert bytes to numpy array efficiently
        data = np.frombuffer(bytes(msg.data), dtype=np.float32)
        data = data.reshape(num_points, point_step // 4)

        # Extract x, y (columns 0, 1) and intensity (column 3) as elevation
        points = data[:, :2].copy()  # x, y
        elevations = data[:, 3].copy()  # intensity as elevation

        self.terrain_points_list.append(points)
        self.terrain_elevations_list.append(elevations)
        self.received_terrain_topics.add(topic_name)

        self.get_logger().info(
            f'Received terrain from {topic_name}: {num_points} points '
            f'({len(self.received_terrain_topics)}/{len(self.terrain_topics)})'
        )

        # Merge when all topics received
        if len(self.received_terrain_topics) >= len(self.terrain_topics):
            self._merge_terrain_data()

    def _merge_terrain_data(self) -> None:
        """Merge terrain data from all topics."""
        self.terrain_points = np.vstack(self.terrain_points_list)
        self.terrain_elevations = np.concatenate(self.terrain_elevations_list)
        self.terrain_received = True
        self.get_logger().info(f'Terrain merged: {len(self.terrain_points)} total points')

        # Build interpolators once (for performance)
        self.get_logger().info('Building interpolators...')
        self._linear_interp = LinearNDInterpolator(self.terrain_points, self.terrain_elevations)
        self._nearest_interp = NearestNDInterpolator(self.terrain_points, self.terrain_elevations)
        self.get_logger().info('Interpolators ready')

        # Unsubscribe from terrain topics to avoid processing more messages
        for sub in self.terrain_subs:
            self.destroy_subscription(sub)
        self.terrain_subs.clear()
        self.get_logger().info('Unsubscribed from terrain topics')

        # Free memory from temporary lists
        self.terrain_points_list.clear()
        self.terrain_elevations_list.clear()

    def _pose_callback(self, msg: Pose2D, robot_id: str) -> None:
        """Record rover position for trajectory."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Sample at specified interval
        last_time = self.last_sample_time.get(robot_id, 0.0)
        if current_time - last_time >= self.sample_interval:
            self.trajectories[robot_id].append((msg.x, msg.y, current_time))
            self.last_sample_time[robot_id] = current_time

    def _get_elevation_at(self, x: float, y: float) -> float:
        """Get terrain elevation at (x, y) using cached interpolators."""
        if self._linear_interp is None:
            return 0.0

        # Use cached linear interpolator
        elevation = self._linear_interp(x, y)

        # Fall back to nearest neighbor if outside convex hull
        if np.isnan(elevation):
            elevation = self._nearest_interp(x, y)

        return float(elevation)

    def _get_output_path(self, base_filename: str) -> str:
        """Generate output file path with timestamp in output directory."""
        name, ext = os.path.splitext(base_filename)
        filename = f'{self.timestamp}_{name}{ext}'
        return os.path.join(self.output_dir, filename)

    def save_data(self, filepath: str = None) -> None:
        """Save terrain and trajectory data to file for later viewing."""
        if filepath is None:
            filepath = self._get_output_path('trajectory_data.npz')
        np.savez(
            filepath,
            terrain_points=self.terrain_points,
            terrain_elevations=self.terrain_elevations,
            trajectories={k: np.array(v) for k, v in self.trajectories.items()},
            robot_ids=self.robot_ids[:self.num_robots],
            elevation_scale=self.elevation_scale,
            z_aspect_ratio=self.z_aspect_ratio,
            terrain_alpha=self.terrain_alpha,
            line_width=self.line_width,
            marker_size=self.marker_size,
            marker_interval=self.marker_interval,
            trajectory_z_offset=self.trajectory_z_offset,
        )
        print(f'[INFO] Data saved to {filepath}')

    def generate_plot(self) -> None:
        """Generate the 3D plot with terrain and trajectories."""
        if not self.terrain_received:
            print('[WARN] No terrain data received. Cannot generate plot.')
            return

        if all(len(traj) == 0 for traj in self.trajectories.values()):
            print('[WARN] No trajectory data recorded. Generating terrain only.')

        # Save data for later viewing
        self.save_data()

        print('[INFO] Generating 3D plot...')

        # Create figure
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Create terrain surface
        self._plot_terrain_surface(ax)

        # Plot trajectories
        self._plot_trajectories(ax)

        # Configure axes
        ax.set_xlabel('X [m]', fontsize=12)
        ax.set_ylabel('Y [m]', fontsize=12)
        ax.set_zlabel('Elevation [m]', fontsize=12)
        ax.set_title('3D Terrain with Rover Trajectories', fontsize=14)

        # Set aspect ratio (X:Y:Z = 1:1:z_aspect_ratio)
        ax.set_box_aspect([1, 1, self.z_aspect_ratio])

        # Add legend
        if any(len(traj) > 0 for traj in self.trajectories.values()):
            ax.legend(loc='upper left')

        # Adjust view angle
        ax.view_init(elev=30, azim=45)

        # Save plot
        plt.tight_layout()
        output_path = self._get_output_path(self.output_file_base)
        plt.savefig(output_path, dpi=self.dpi, bbox_inches='tight')
        print(f'[INFO] Plot saved to {output_path}')

        # Display plot (blocking - user closes window to exit)
        plt.show()

    def _plot_terrain_surface(self, ax: Axes3D) -> None:
        """Plot terrain as a 3D surface."""
        # Create regular grid for surface plot
        x_unique = np.unique(self.terrain_points[:, 0])
        y_unique = np.unique(self.terrain_points[:, 1])

        # Determine grid resolution
        x_min, x_max = x_unique.min(), x_unique.max()
        y_min, y_max = y_unique.min(), y_unique.max()

        # Create meshgrid
        grid_resolution = min(len(x_unique), len(y_unique), 100)
        xi = np.linspace(x_min, x_max, grid_resolution)
        yi = np.linspace(y_min, y_max, grid_resolution)
        X, Y = np.meshgrid(xi, yi)

        # Interpolate elevation data onto grid
        Z = griddata(
            self.terrain_points,
            self.terrain_elevations * self.elevation_scale,
            (X, Y),
            method='linear',
            fill_value=0.0
        )

        # Plot surface
        surf = ax.plot_surface(
            X, Y, Z,
            cmap=cm.terrain,
            alpha=self.terrain_alpha,
            linewidth=0,
            antialiased=True
        )

        # Add colorbar
        fig = ax.get_figure()
        fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Elevation [m]')

    def _plot_trajectories(self, ax: Axes3D) -> None:
        """Plot rover trajectories on the terrain surface."""
        for idx, robot_id in enumerate(self.robot_ids[:self.num_robots]):
            trajectory = self.trajectories.get(robot_id, [])
            if len(trajectory) == 0:
                continue

            color = self.robot_colors[idx % len(self.robot_colors)]

            # Extract x, y coordinates
            xs = [p[0] for p in trajectory]
            ys = [p[1] for p in trajectory]

            # Get z (elevation) for each point on trajectory
            zs = [self._get_elevation_at(x, y) * self.elevation_scale + self.trajectory_z_offset
                  for x, y in zip(xs, ys)]

            # Determine if this robot should have lines
            # show_lines=True and (line_robot_ids empty or robot_id in line_robot_ids)
            should_show_line = self.show_lines and (
                len(self.line_robot_ids) == 0 or robot_id in self.line_robot_ids
            )

            # Plot trajectory line (optional)
            if should_show_line:
                ax.plot(xs, ys, zs,
                       color=color,
                       linewidth=self.line_width,
                       label=f'Robot {robot_id}')
                label = None  # Already labeled by line
            else:
                label = f'Robot {robot_id}'

            # Plot markers at intervals
            marker_xs = xs[::self.marker_interval]
            marker_ys = ys[::self.marker_interval]
            marker_zs = zs[::self.marker_interval]

            ax.scatter(marker_xs, marker_ys, marker_zs,
                      color=color,
                      s=self.marker_size,
                      marker='o',
                      edgecolors='black',
                      linewidths=0.5,
                      label=label)

            # Mark start and end points
            if len(xs) > 0:
                # Start point (triangle)
                ax.scatter([xs[0]], [ys[0]], [zs[0]],
                          color=color,
                          s=self.marker_size * 2,
                          marker='^',
                          edgecolors='black',
                          linewidths=1,
                          zorder=10)
                # End point (square)
                ax.scatter([xs[-1]], [ys[-1]], [zs[-1]],
                          color=color,
                          s=self.marker_size * 2,
                          marker='s',
                          edgecolors='black',
                          linewidths=1,
                          zorder=10)

            print(f'[INFO] Plotted {len(trajectory)} points for {robot_id}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlotter3D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Generate plot after spin ends
    print('[INFO] Generating plot...')
    node.generate_plot()
    print('[INFO] To reopen: ros2 run sensor_field trajectory_viewer')

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
