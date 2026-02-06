#!/usr/bin/env python3
"""
Viewer for saved trajectory data from trajectory_plotter_3d.

Usage:
    ros2 run sensor_field trajectory_viewer [data_file.npz]

Default file: trajectory_data.npz
"""

import sys

# Fix mpl_toolkits namespace package conflict
_user_site = '/home/neo/.local/lib/python3.10/site-packages'
sys.path.insert(0, _user_site)
for _mod in [k for k in list(sys.modules.keys()) if 'mpl_toolkits' in k or 'matplotlib' in k]:
    del sys.modules[_mod]

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.interpolate import griddata


def load_data(filepath: str) -> dict:
    """Load trajectory data from npz file."""
    data = np.load(filepath, allow_pickle=True)
    result = {
        'terrain_points': data['terrain_points'],
        'terrain_elevations': data['terrain_elevations'],
        'trajectories': data['trajectories'].item(),
        'robot_ids': list(data['robot_ids']),
        'elevation_scale': float(data['elevation_scale']),
        'terrain_alpha': float(data['terrain_alpha']),
        'line_width': float(data['line_width']),
        'marker_size': float(data['marker_size']),
        'marker_interval': int(data['marker_interval']),
    }
    # Handle optional trajectory_z_offset (for backward compatibility)
    if 'trajectory_z_offset' in data:
        result['trajectory_z_offset'] = float(data['trajectory_z_offset'])
    else:
        result['trajectory_z_offset'] = 2.0

    # Handle coordinate scaling parameters (for backward compatibility)
    if 'sensor_distance_scale' in data:
        result['sensor_distance_scale'] = float(data['sensor_distance_scale'])
    else:
        result['sensor_distance_scale'] = 1.0

    if 'original_coordinate_unit' in data:
        result['original_coordinate_unit'] = str(data['original_coordinate_unit'])
    else:
        result['original_coordinate_unit'] = 'm'

    if 'original_data_unit' in data:
        result['original_data_unit'] = str(data['original_data_unit'])
    else:
        result['original_data_unit'] = ''

    # Calculate display scale (inverse of sensor_distance_scale)
    result['display_scale_xy'] = 1.0 / result['sensor_distance_scale']

    return result


def plot_terrain_surface(ax, data: dict) -> None:
    """Plot terrain as a 3D surface."""
    terrain_points = data['terrain_points']
    terrain_elevations = data['terrain_elevations']
    elevation_scale = data['elevation_scale']
    terrain_alpha = data['terrain_alpha']
    display_scale_xy = data['display_scale_xy']

    # Apply display scale to terrain coordinates
    scaled_points = terrain_points.copy()
    scaled_points[:, 0] *= display_scale_xy
    scaled_points[:, 1] *= display_scale_xy

    x_unique = np.unique(scaled_points[:, 0])
    y_unique = np.unique(scaled_points[:, 1])

    x_min, x_max = x_unique.min(), x_unique.max()
    y_min, y_max = y_unique.min(), y_unique.max()

    grid_resolution = min(len(x_unique), len(y_unique), 100)
    xi = np.linspace(x_min, x_max, grid_resolution)
    yi = np.linspace(y_min, y_max, grid_resolution)
    X, Y = np.meshgrid(xi, yi)

    Z = griddata(
        scaled_points,
        terrain_elevations * elevation_scale,
        (X, Y),
        method='linear',
        fill_value=0.0
    )

    surf = ax.plot_surface(
        X, Y, Z,
        cmap=cm.terrain,
        alpha=terrain_alpha,
        linewidth=0,
        antialiased=True
    )

    fig = ax.get_figure()
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Elevation [m]')


def get_elevation_at(terrain_points, terrain_elevations, x: float, y: float) -> float:
    """Get terrain elevation at (x, y)."""
    distances = np.sqrt((terrain_points[:, 0] - x)**2 + (terrain_points[:, 1] - y)**2)
    nearest_idx = np.argmin(distances)
    return terrain_elevations[nearest_idx]


def plot_trajectories(ax, data: dict) -> None:
    """Plot rover trajectories on the terrain surface."""
    robot_colors = ['red', 'blue', 'green', 'orange', 'purple',
                    'cyan', 'magenta', 'yellow', 'brown', 'pink']

    trajectories = data['trajectories']
    robot_ids = data['robot_ids']
    terrain_points = data['terrain_points']
    terrain_elevations = data['terrain_elevations']
    elevation_scale = data['elevation_scale']
    line_width = data['line_width']
    marker_size = data['marker_size']
    marker_interval = data['marker_interval']
    trajectory_z_offset = data['trajectory_z_offset']
    display_scale_xy = data['display_scale_xy']

    for idx, robot_id in enumerate(robot_ids):
        trajectory = trajectories.get(robot_id, np.array([]))
        if len(trajectory) == 0:
            continue

        color = robot_colors[idx % len(robot_colors)]

        xs = trajectory[:, 0]
        ys = trajectory[:, 1]

        # Apply display scale to trajectory coordinates
        xs_scaled = xs * display_scale_xy
        ys_scaled = ys * display_scale_xy

        zs = [get_elevation_at(terrain_points, terrain_elevations, x, y) * elevation_scale + trajectory_z_offset
              for x, y in zip(xs, ys)]

        ax.plot(xs_scaled, ys_scaled, zs,
                color=color,
                linewidth=line_width,
                label=f'Robot {robot_id}')

        marker_xs = xs_scaled[::marker_interval]
        marker_ys = ys_scaled[::marker_interval]
        marker_zs = zs[::marker_interval]

        ax.scatter(marker_xs, marker_ys, marker_zs,
                   color=color,
                   s=marker_size,
                   marker='o',
                   edgecolors='black',
                   linewidths=0.5)

        if len(xs_scaled) > 0:
            ax.scatter([xs_scaled[0]], [ys_scaled[0]], [zs[0]],
                       color=color,
                       s=marker_size * 2,
                       marker='^',
                       edgecolors='black',
                       linewidths=1,
                       zorder=10)
            ax.scatter([xs_scaled[-1]], [ys_scaled[-1]], [zs[-1]],
                       color=color,
                       s=marker_size * 2,
                       marker='s',
                       edgecolors='black',
                       linewidths=1,
                       zorder=10)

        print(f'[INFO] Plotted {len(trajectory)} points for {robot_id}')


def plot_contour(ax, data: dict, num_levels: int = 20) -> None:
    """Plot terrain as 2D contour map."""
    terrain_points = data['terrain_points']
    terrain_elevations = data['terrain_elevations']
    display_scale_xy = data['display_scale_xy']

    # Apply display scale to terrain coordinates
    scaled_points = terrain_points.copy()
    scaled_points[:, 0] *= display_scale_xy
    scaled_points[:, 1] *= display_scale_xy

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
        terrain_elevations,
        (X, Y),
        method='linear',
        fill_value=0.0
    )

    # Filled contour
    contourf = ax.contourf(X, Y, Z, levels=num_levels, cmap='YlOrRd', alpha=0.8)

    # Contour lines
    contour = ax.contour(X, Y, Z, levels=num_levels, colors='black', linewidths=0.3, alpha=0.5)
    ax.clabel(contour, inline=True, fontsize=6, fmt='%.2f')

    fig = ax.get_figure()
    fig.colorbar(contourf, ax=ax, shrink=0.8, label='Radiation (normalized)')


def plot_trajectories_2d(ax, data: dict) -> None:
    """Plot rover trajectories on 2D map."""
    robot_colors = ['red', 'blue', 'green', 'orange', 'purple',
                    'cyan', 'magenta', 'yellow', 'brown', 'pink']

    trajectories = data['trajectories']
    robot_ids = data['robot_ids']
    line_width = data['line_width']
    marker_size = data['marker_size']
    marker_interval = data['marker_interval']
    display_scale_xy = data['display_scale_xy']

    for idx, robot_id in enumerate(robot_ids):
        trajectory = trajectories.get(robot_id, np.array([]))
        if len(trajectory) == 0:
            continue

        color = robot_colors[idx % len(robot_colors)]

        xs = trajectory[:, 0]
        ys = trajectory[:, 1]

        # Apply display scale to trajectory coordinates
        xs_scaled = xs * display_scale_xy
        ys_scaled = ys * display_scale_xy

        # Trajectory line
        ax.plot(xs_scaled, ys_scaled, color=color, linewidth=line_width, label=f'Robot {robot_id}', zorder=5)

        # Markers at intervals
        marker_xs = xs_scaled[::marker_interval]
        marker_ys = ys_scaled[::marker_interval]
        ax.scatter(marker_xs, marker_ys, color=color, s=marker_size,
                   marker='o', edgecolors='black', linewidths=0.5, zorder=6)

        if len(xs_scaled) > 0:
            # Start (triangle)
            ax.scatter([xs_scaled[0]], [ys_scaled[0]], color=color, s=marker_size * 3,
                       marker='^', edgecolors='black', linewidths=1, zorder=10)
            # End (square)
            ax.scatter([xs_scaled[-1]], [ys_scaled[-1]], color=color, s=marker_size * 3,
                       marker='s', edgecolors='black', linewidths=1, zorder=10)

        print(f'[INFO] Plotted {len(trajectory)} points for {robot_id}')


def generate_plot(data: dict, mode: str = '3d', num_levels: int = 20) -> None:
    """Generate the plot.

    Args:
        data: Trajectory data dict
        mode: '3d' for 3D surface, 'contour' for 2D contour map
        num_levels: Number of contour levels (only for contour mode)
    """
    coord_unit = data['original_coordinate_unit']
    data_unit = data['original_data_unit']
    data_label = f'Radiation [{data_unit}]' if data_unit else 'Radiation (normalized)'

    if mode == 'contour':
        fig, ax = plt.subplots(figsize=(12, 10), constrained_layout=True)

        plot_contour(ax, data, num_levels)
        plot_trajectories_2d(ax, data)

        ax.set_xlabel(f'X [{coord_unit}]', fontsize=12)
        ax.set_ylabel(f'Y [{coord_unit}]', fontsize=12)
        ax.set_title('Radiation Field with Rover Trajectories', fontsize=14)
        ax.set_aspect('equal')

        if any(len(t) > 0 for t in data['trajectories'].values()):
            ax.legend(loc='upper left')

    else:  # 3d
        fig = plt.figure(figsize=(14, 10), constrained_layout=True)
        ax = fig.add_subplot(111, projection='3d')

        plot_terrain_surface(ax, data)
        plot_trajectories(ax, data)

        ax.set_xlabel(f'X [{coord_unit}]', fontsize=12)
        ax.set_ylabel(f'Y [{coord_unit}]', fontsize=12)
        ax.set_zlabel(data_label, fontsize=12)
        ax.set_title('3D Terrain with Rover Trajectories', fontsize=14)

        if any(len(t) > 0 for t in data['trajectories'].values()):
            ax.legend(loc='upper left')

        ax.view_init(elev=30, azim=45)

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='View saved trajectory data')
    parser.add_argument('file', nargs='?', default='trajectory_data.npz',
                        help='Path to trajectory data file (default: trajectory_data.npz)')
    parser.add_argument('-z', '--z-offset', type=float, default=None,
                        help='Override trajectory Z offset (default: use saved value)')
    parser.add_argument('-m', '--mode', choices=['3d', 'contour'], default='3d',
                        help='Visualization mode: 3d or contour (default: 3d)')
    parser.add_argument('-l', '--levels', type=int, default=20,
                        help='Number of contour levels (default: 20)')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Save plot to file instead of displaying')
    parser.add_argument('-s', '--scale', type=float, default=None,
                        help='Override sensor_distance_scale (default: use saved value)')
    parser.add_argument('--coord-unit', type=str, default=None,
                        help='Override coordinate unit label (default: use saved value)')
    parser.add_argument('--data-unit', type=str, default=None,
                        help='Override data unit label (default: use saved value)')
    args = parser.parse_args()

    print(f'[INFO] Loading data from {args.file}...')
    try:
        data = load_data(args.file)
    except FileNotFoundError:
        print(f'[ERROR] File not found: {args.file}')
        sys.exit(1)

    # Override z offset if specified
    if args.z_offset is not None:
        data['trajectory_z_offset'] = args.z_offset
        print(f'[INFO] Using Z offset: {args.z_offset}')

    # Override scale parameters if specified
    if args.scale is not None:
        data['sensor_distance_scale'] = args.scale
        data['display_scale_xy'] = 1.0 / args.scale
        print(f'[INFO] Overriding sensor_distance_scale: {args.scale}')

    if args.coord_unit is not None:
        data['original_coordinate_unit'] = args.coord_unit
        print(f'[INFO] Overriding coordinate unit: {args.coord_unit}')

    if args.data_unit is not None:
        data['original_data_unit'] = args.data_unit
        print(f'[INFO] Overriding data unit: {args.data_unit}')

    print(f'[INFO] Generating {args.mode} plot...')
    generate_plot(data, mode=args.mode, num_levels=args.levels)

    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches='tight')
        print(f'[INFO] Saved to {args.output}')


if __name__ == '__main__':
    main()
