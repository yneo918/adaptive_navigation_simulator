"""
ROS 2 Node for 3D terrain visualization with rover trajectories.

Subscribes to:
- sensor_field/points (PointCloud2): Terrain elevation data
- /sim/p{i}/pose2D (Pose2D): Rover positions

Creates a matplotlib 3D plot showing:
- Terrain as a surface
- Rover trajectories as lines and points on the terrain surface
"""

import struct
import sys
from collections import defaultdict
from typing import Dict, List, Tuple

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
from scipy.interpolate import griddata

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
        self.declare_parameter('output_file', 'trajectory_3d.png')
        self.declare_parameter('dpi', 150)
        self.declare_parameter('elevation_scale', 1.0)  # Scale factor for Z axis
        self.declare_parameter('terrain_alpha', 0.7)
        self.declare_parameter('line_width', 2.0)
        self.declare_parameter('marker_size', 10)
        self.declare_parameter('marker_interval', 10)  # Plot marker every N points
        self.declare_parameter('trajectory_z_offset', 2.0)  # Z offset to lift trajectory above terrain

        # Get parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.robot_ids = list(self.get_parameter('robot_ids').value)
        self.sample_interval = self.get_parameter('trajectory_sample_interval').value
        self.output_file = self.get_parameter('output_file').value
        self.dpi = self.get_parameter('dpi').value
        self.elevation_scale = self.get_parameter('elevation_scale').value
        self.terrain_alpha = self.get_parameter('terrain_alpha').value
        self.line_width = self.get_parameter('line_width').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_interval = self.get_parameter('marker_interval').value
        self.trajectory_z_offset = self.get_parameter('trajectory_z_offset').value

        # Terrain data storage
        self.terrain_points: np.ndarray = None
        self.terrain_elevations: np.ndarray = None
        self.terrain_received = False

        # Trajectory storage: robot_id -> list of (x, y, timestamp)
        self.trajectories: Dict[str, List[Tuple[float, float, float]]] = defaultdict(list)
        self.last_sample_time: Dict[str, float] = {}

        # Colors for different robots
        self.robot_colors = ['red', 'blue', 'green', 'orange', 'purple',
                             'cyan', 'magenta', 'yellow', 'brown', 'pink']

        # Subscribe to terrain data
        self.terrain_sub = self.create_subscription(
            PointCloud2,
            'sensor_field/points',
            self._terrain_callback,
            10
        )

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

        self.get_logger().info('TrajectoryPlotter3D initialized. Press Ctrl+C to generate plot.')

    def _terrain_callback(self, msg: PointCloud2) -> None:
        """Parse terrain PointCloud2 message."""
        if self.terrain_received:
            return

        point_step = msg.point_step
        num_points = msg.width * msg.height

        points = []
        elevations = []

        for i in range(num_points):
            offset = i * point_step
            x, y, z, intensity = struct.unpack_from('ffff', msg.data, offset)
            points.append((x, y))
            elevations.append(intensity)  # Use intensity as the elevation value

        self.terrain_points = np.array(points)
        self.terrain_elevations = np.array(elevations)
        self.terrain_received = True
        self.get_logger().info(f'Received terrain data: {num_points} points')

    def _pose_callback(self, msg: Pose2D, robot_id: str) -> None:
        """Record rover position for trajectory."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Sample at specified interval
        last_time = self.last_sample_time.get(robot_id, 0.0)
        if current_time - last_time >= self.sample_interval:
            self.trajectories[robot_id].append((msg.x, msg.y, current_time))
            self.last_sample_time[robot_id] = current_time

    def _get_elevation_at(self, x: float, y: float) -> float:
        """Get terrain elevation at (x, y) using interpolation."""
        if self.terrain_points is None or len(self.terrain_points) == 0:
            return 0.0

        # Use nearest neighbor interpolation for efficiency
        distances = np.sqrt((self.terrain_points[:, 0] - x)**2 +
                           (self.terrain_points[:, 1] - y)**2)
        nearest_idx = np.argmin(distances)
        return self.terrain_elevations[nearest_idx]

    def save_data(self, filepath: str = 'trajectory_data.npz') -> None:
        """Save terrain and trajectory data to file for later viewing."""
        np.savez(
            filepath,
            terrain_points=self.terrain_points,
            terrain_elevations=self.terrain_elevations,
            trajectories={k: np.array(v) for k, v in self.trajectories.items()},
            robot_ids=self.robot_ids[:self.num_robots],
            elevation_scale=self.elevation_scale,
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

        # Add legend
        if any(len(traj) > 0 for traj in self.trajectories.values()):
            ax.legend(loc='upper left')

        # Adjust view angle
        ax.view_init(elev=30, azim=45)

        # Save plot
        plt.tight_layout()
        plt.savefig(self.output_file, dpi=self.dpi, bbox_inches='tight')
        print(f'[INFO] Plot saved to {self.output_file}')

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

            # Plot trajectory line
            ax.plot(xs, ys, zs,
                   color=color,
                   linewidth=self.line_width,
                   label=f'Robot {robot_id}')

            # Plot markers at intervals
            marker_xs = xs[::self.marker_interval]
            marker_ys = ys[::self.marker_interval]
            marker_zs = zs[::self.marker_interval]

            ax.scatter(marker_xs, marker_ys, marker_zs,
                      color=color,
                      s=self.marker_size,
                      marker='o',
                      edgecolors='black',
                      linewidths=0.5)

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
