"""
ROS 2 Node for 3D terrain visualization with rover trajectories.

Subscribes to:
- sensor_field/points (PointCloud2): Terrain elevation data
- /sim/p{i}/pose2D (Pose2D): Rover positions

Creates a matplotlib 3D plot showing:
- Terrain as a surface
- Rover trajectories as lines and points on the terrain surface
"""

import math
import os
import struct
import sys
from collections import defaultdict
from datetime import datetime
from functools import partial
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
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
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
        # Coordinate scaling parameters (for reverting sensor_field distance_scale)
        self.declare_parameter('sensor_distance_scale', 1.0)  # Same value as sensor_field distance_scale
        self.declare_parameter('original_coordinate_unit', 'm')  # Original coordinate unit (e.g., 'm', 'km', 'mm')
        self.declare_parameter('original_data_unit', '')  # Original data unit for Z axis (e.g., 'dBm', 'Sv/h')
        self.declare_parameter('terrain_alpha', 0.7)
        self.declare_parameter('line_width', 2.0)
        self.declare_parameter('marker_size', 10)
        self.declare_parameter('marker_interval', 10)  # Plot marker every N points
        self.declare_parameter('trajectory_z_offset', 0.0)  # Z offset to lift trajectory above terrain
        self.declare_parameter('terrain_topics', ['sensor_field/points'])
        self.declare_parameter('show_lines', True)  # Show trajectory lines
        self.declare_parameter('line_robot_ids', ['p1', 'p2', 'p3', 'p4', 'p5'])  # Robot IDs to show lines (empty = all if show_lines=True)
        self.declare_parameter('visualization_mode', 'contour')  # '3d', 'contour', or 'both'
        self.declare_parameter('show_plot', True)  # set False for headless / batch runs
        self.declare_parameter('contour_levels', 20)  # Number of contour levels

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

        # Coordinate scaling parameters
        self.sensor_distance_scale = self.get_parameter('sensor_distance_scale').value
        self.original_coord_unit = self.get_parameter('original_coordinate_unit').value
        self.original_data_unit = self.get_parameter('original_data_unit').value

        # Calculate display scale (inverse of sensor_distance_scale)
        if self.sensor_distance_scale > 0.0:
            self.display_scale_xy = 1.0 / self.sensor_distance_scale
        else:
            self.display_scale_xy = 1.0
        self.line_width = self.get_parameter('line_width').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_interval = self.get_parameter('marker_interval').value
        self.trajectory_z_offset = self.get_parameter('trajectory_z_offset').value
        self.terrain_topics: List[str] = list(self.get_parameter('terrain_topics').value)
        self.show_lines = self.get_parameter('show_lines').value
        self.line_robot_ids: List[str] = list(self.get_parameter('line_robot_ids').value)
        self.visualization_mode = self.get_parameter('visualization_mode').value
        self.show_plot = self.get_parameter('show_plot').value
        self.contour_levels = self.get_parameter('contour_levels').value

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
                partial(self._terrain_callback, topic_name=topic),
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
                partial(self._pose_callback, robot_id=robot_id),
                10
            )
            self.pose_subs.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

        # Service client for elevation queries
        self.elevation_client = self.create_client(GetSensor2D, 'get_sensor')

        # RViz用のPathパブリッシャーとメッセージを各ロボット用に作成
        self.path_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self.paths: Dict[str, Path] = {}

        # line_robot_idsで指定されたロボットのみPathパブリッシャーを作成
        robot_ids_for_path = self.line_robot_ids if len(self.line_robot_ids) > 0 else self.robot_ids[:self.num_robots]
        for robot_id in robot_ids_for_path:
            topic = f'{self.robot_prefix}/{robot_id}/path' if self.robot_prefix else f'/{robot_id}/path'
            pub = self.create_publisher(Path, topic, 10)
            self.path_publishers[robot_id] = pub

            # Path初期化
            path = Path()
            path.header.frame_id = 'world'
            self.paths[robot_id] = path

            self.get_logger().info(f'Publishing RViz path to {topic}')

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
        self.get_logger().info(f'  visualization_mode: {self.visualization_mode}')
        self.get_logger().info(f'  sensor_distance_scale: {self.sensor_distance_scale}')
        self.get_logger().info(f'  original_coordinate_unit: {self.original_coord_unit}')
        self.get_logger().info(f'  original_data_unit: {self.original_data_unit}')
        self.get_logger().info(f'  display_scale_xy: {self.display_scale_xy}')
        self.get_logger().info(f'  contour_levels: {self.contour_levels}')
        if len(robot_ids_for_path) > 0:
            self.get_logger().info(f'  RViz path enabled for: {robot_ids_for_path}')
        else:
            self.get_logger().info('  RViz path: disabled (no robots in line_robot_ids)')
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

            # RViz Path更新（line_robot_idsに含まれる場合のみ）
            if robot_id in self.path_publishers:
                self._update_rviz_path(robot_id, msg)

    def _update_rviz_path(self, robot_id: str, msg: Pose2D) -> None:
        """Update and publish RViz Path message."""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'world'

        # Position
        pose_stamped.pose.position.x = msg.x
        pose_stamped.pose.position.y = msg.y
        pose_stamped.pose.position.z = 0.0

        # Orientation (theta → quaternion変換、REP103準拠: Z軸回転)
        # q = [0, 0, sin(θ/2), cos(θ/2)]
        half_theta = msg.theta / 2.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(half_theta)
        pose_stamped.pose.orientation.w = math.cos(half_theta)

        # Pathに追加（無制限）
        self.paths[robot_id].poses.append(pose_stamped)
        self.paths[robot_id].header.stamp = pose_stamped.header.stamp

        # 配信
        self.path_publishers[robot_id].publish(self.paths[robot_id])

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
        # Terrain may legitimately be absent (field node not ready); the
        # trajectories are still valid data, so save with empty terrain
        # arrays rather than dropping the whole run.
        terrain_points = (self.terrain_points
                          if self.terrain_points is not None
                          else np.zeros((0, 2)))
        terrain_elevations = (self.terrain_elevations
                              if self.terrain_elevations is not None
                              else np.zeros(0))
        np.savez(
            filepath,
            terrain_points=terrain_points,
            terrain_elevations=terrain_elevations,
            trajectories={k: np.array(v) for k, v in self.trajectories.items()},
            robot_ids=self.robot_ids[:self.num_robots],
            elevation_scale=self.elevation_scale,
            z_aspect_ratio=self.z_aspect_ratio,
            terrain_alpha=self.terrain_alpha,
            line_width=self.line_width,
            marker_size=self.marker_size,
            marker_interval=self.marker_interval,
            trajectory_z_offset=self.trajectory_z_offset,
            # Coordinate scaling information
            sensor_distance_scale=self.sensor_distance_scale,
            original_coordinate_unit=self.original_coord_unit,
            original_data_unit=self.original_data_unit,
        )
        print(f'[INFO] Data saved to {filepath}')

    def generate_plot(self) -> None:
        """Generate plot(s) based on visualization_mode."""
        if all(len(traj) == 0 for traj in self.trajectories.values()):
            print('[WARN] No trajectory data recorded. Generating terrain only.')

        # Save data for later viewing. This must happen BEFORE the terrain
        # guard: trajectories are valid data even when the terrain cloud was
        # never received, and dropping them silently loses the whole run.
        self.save_data()

        if not self.terrain_received:
            print('[WARN] No terrain data received. Cannot generate plot '
                  '(NPZ saved without terrain).')
            return

        if self.visualization_mode == 'both':
            # Generate both 3D and contour plots
            print('[INFO] Generating 3D plot...')
            self._generate_3d_plot()
            print('[INFO] Generating contour plot...')
            self._generate_contour_plot()
        elif self.visualization_mode == 'contour':
            # Generate contour plot only
            print('[INFO] Generating contour plot...')
            self._generate_contour_plot()
        else:  # '3d'
            # Generate 3D plot only
            print('[INFO] Generating 3D plot...')
            self._generate_3d_plot()

    def _generate_3d_plot(self) -> None:
        """Generate 3D surface plot."""
        # Create figure with constrained layout for zoom compatibility
        fig = plt.figure(figsize=(14, 10), constrained_layout=True)
        ax = fig.add_subplot(111, projection='3d')

        # Create terrain surface
        self._plot_terrain_surface(ax)

        # Plot trajectories
        self._plot_trajectories(ax)

        # Configure axes with dynamic units
        ax.set_xlabel(f'X [{self.original_coord_unit}]', fontsize=12)
        ax.set_ylabel(f'Y [{self.original_coord_unit}]', fontsize=12)
        if self.original_data_unit:
            ax.set_zlabel(f'Radiation [{self.original_data_unit}]', fontsize=12)
        else:
            ax.set_zlabel('Radiation []', fontsize=12)
        ax.set_title('3D Terrain with Rover Trajectories', fontsize=14)

        # Set aspect ratio (X:Y:Z = 1:1:z_aspect_ratio)
        ax.set_box_aspect([1, 1, self.z_aspect_ratio])

        # Add legend
        if any(len(traj) > 0 for traj in self.trajectories.values()):
            ax.legend(loc='upper left')

        # Adjust view angle
        ax.view_init(elev=30, azim=45)

        # Save plot
        name, ext = os.path.splitext(self.output_file_base)
        output_file_3d = f'{name}_3d{ext}'
        output_path = self._get_output_path(output_file_3d)
        plt.savefig(output_path, dpi=self.dpi, bbox_inches='tight')
        print(f'[INFO] 3D plot saved to {output_path}')

        if self.show_plot:
            # Display plot (blocking - user closes window to exit)
            plt.show()
        else:
            plt.close(fig)

    def _generate_contour_plot(self) -> None:
        """Generate 2D contour plot."""
        # Create figure with constrained layout for zoom compatibility
        fig, ax = plt.subplots(figsize=(12, 10), constrained_layout=True)

        # Plot contour
        self._plot_contour(ax)

        # Plot trajectories
        self._plot_trajectories_2d(ax)

        # Configure axes with dynamic units
        ax.set_xlabel(f'X [{self.original_coord_unit}]', fontsize=12)
        ax.set_ylabel(f'Y [{self.original_coord_unit}]', fontsize=12)
        ax.set_title('Radiation Field with Rover Trajectories', fontsize=14)
        ax.set_aspect('equal')

        # Add legend
        if any(len(traj) > 0 for traj in self.trajectories.values()):
            ax.legend(loc='upper left')

        # Save plot
        name, ext = os.path.splitext(self.output_file_base)
        output_file_contour = f'{name}_contour{ext}'
        output_path = self._get_output_path(output_file_contour)
        plt.savefig(output_path, dpi=self.dpi, bbox_inches='tight')
        print(f'[INFO] Contour plot saved to {output_path}')

        if self.show_plot:
            # Display plot (blocking - user closes window to exit)
            plt.show()
        else:
            plt.close(fig)

    def _plot_terrain_surface(self, ax: Axes3D) -> None:
        """Plot terrain as a 3D surface."""
        # Apply display scale to coordinates
        scaled_points = self.terrain_points.copy()
        scaled_points[:, 0] *= self.display_scale_xy
        scaled_points[:, 1] *= self.display_scale_xy

        # Create regular grid for surface plot
        x_unique = np.unique(scaled_points[:, 0])
        y_unique = np.unique(scaled_points[:, 1])

        # Determine grid resolution
        x_min, x_max = x_unique.min(), x_unique.max()
        y_min, y_max = y_unique.min(), y_unique.max()

        # Create meshgrid
        grid_resolution = min(len(x_unique), len(y_unique), 100)
        xi = np.linspace(x_min, x_max, grid_resolution)
        yi = np.linspace(y_min, y_max, grid_resolution)
        X, Y = np.meshgrid(xi, yi)

        # Interpolate elevation data onto grid (use scaled coordinates)
        Z = griddata(
            scaled_points,
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

    def _plot_contour(self, ax) -> None:
        """Plot terrain as 2D contour map."""
        # Apply display scale to coordinates
        scaled_points = self.terrain_points.copy()
        scaled_points[:, 0] *= self.display_scale_xy
        scaled_points[:, 1] *= self.display_scale_xy

        x_unique = np.unique(scaled_points[:, 0])
        y_unique = np.unique(scaled_points[:, 1])

        x_min, x_max = x_unique.min(), x_unique.max()
        y_min, y_max = y_unique.min(), y_unique.max()

        grid_resolution = min(len(x_unique), len(y_unique), 200)
        xi = np.linspace(x_min, x_max, grid_resolution)
        yi = np.linspace(y_min, y_max, grid_resolution)
        X, Y = np.meshgrid(xi, yi)

        Z = griddata(
            scaled_points,
            self.terrain_elevations,
            (X, Y),
            method='linear',
            fill_value=0.0
        )

        # Filled contour
        contourf = ax.contourf(X, Y, Z, levels=self.contour_levels, cmap='YlOrRd', alpha=0.8)

        # Contour lines
        contour = ax.contour(X, Y, Z, levels=self.contour_levels, colors='black', linewidths=0.3, alpha=0.5)
        ax.clabel(contour, inline=True, fontsize=6, fmt='%.2f')

        fig = ax.get_figure()
        fig.colorbar(contourf, ax=ax, shrink=0.8, label='Radiation (normalized)')

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

            # Get z (elevation) for each point on trajectory (using original coordinates)
            zs = [self._get_elevation_at(x, y) * self.elevation_scale + self.trajectory_z_offset
                  for x, y in zip(xs, ys)]

            # Apply display scale to coordinates for plotting
            xs = [x * self.display_scale_xy for x in xs]
            ys = [y * self.display_scale_xy for y in ys]

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

    def _plot_trajectories_2d(self, ax) -> None:
        """Plot rover trajectories on 2D map."""
        for idx, robot_id in enumerate(self.robot_ids[:self.num_robots]):
            trajectory = self.trajectories.get(robot_id, [])
            if len(trajectory) == 0:
                continue

            color = self.robot_colors[idx % len(self.robot_colors)]

            # Extract x, y coordinates
            xs = [p[0] for p in trajectory]
            ys = [p[1] for p in trajectory]

            # Apply display scale to coordinates for plotting
            xs = [x * self.display_scale_xy for x in xs]
            ys = [y * self.display_scale_xy for y in ys]

            # Determine if this robot should have lines
            should_show_line = self.show_lines and (
                len(self.line_robot_ids) == 0 or robot_id in self.line_robot_ids
            )

            # Plot trajectory line (optional)
            if should_show_line:
                ax.plot(xs, ys, color=color, linewidth=self.line_width,
                       label=f'Robot {robot_id}', zorder=5)
                label = None  # Already labeled by line
            else:
                label = f'Robot {robot_id}'

            # Markers at intervals
            marker_xs = xs[::self.marker_interval]
            marker_ys = ys[::self.marker_interval]
            ax.scatter(marker_xs, marker_ys, color=color, s=self.marker_size,
                      marker='o', edgecolors='black', linewidths=0.5,
                      zorder=6, label=label)

            if len(xs) > 0:
                # Start (triangle)
                ax.scatter([xs[0]], [ys[0]], color=color, s=self.marker_size * 3,
                          marker='^', edgecolors='black', linewidths=1, zorder=10)
                # End (square)
                ax.scatter([xs[-1]], [ys[-1]], color=color, s=self.marker_size * 3,
                          marker='s', edgecolors='black', linewidths=1, zorder=10)

            print(f'[INFO] Plotted {len(trajectory)} points for {robot_id}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlotter3D()

    spin_error = None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # DDS / pybind11 errors can occur during shutdown when the executor
        # is mid-callback. We must not let these prevent data persistence.
        spin_error = e
        print(f'[WARN] spin raised {type(e).__name__}: {e}', file=sys.stderr)
        print('[WARN] proceeding to save data anyway', file=sys.stderr)

    # Generate plot (which saves NPZ). Even if plot rendering fails, fall
    # back to a direct save_data() so trajectories are preserved.
    print('[INFO] Generating plot...')
    try:
        node.generate_plot()
    except Exception as e:
        print(f'[ERROR] generate_plot failed: {type(e).__name__}: {e}',
              file=sys.stderr)
        try:
            if getattr(node, 'terrain_received', False):
                node.save_data()
                print('[INFO] Fallback: NPZ saved without plot')
            else:
                print('[ERROR] no terrain received; NPZ cannot be saved',
                      file=sys.stderr)
        except Exception as e2:
            print(f'[ERROR] save_data fallback failed: {e2}',
                  file=sys.stderr)
    print('[INFO] To reopen: ros2 run sensor_field trajectory_viewer')

    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass

    # If spin died with a non-KeyboardInterrupt error, still indicate failure
    # so the orchestrator's stop_plotter status flags this run for review.
    if spin_error is not None:
        sys.exit(2)


if __name__ == '__main__':
    main()
