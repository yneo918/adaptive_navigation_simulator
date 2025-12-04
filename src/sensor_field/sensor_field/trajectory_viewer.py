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
    return result


def plot_terrain_surface(ax, data: dict) -> None:
    """Plot terrain as a 3D surface."""
    terrain_points = data['terrain_points']
    terrain_elevations = data['terrain_elevations']
    elevation_scale = data['elevation_scale']
    terrain_alpha = data['terrain_alpha']

    x_unique = np.unique(terrain_points[:, 0])
    y_unique = np.unique(terrain_points[:, 1])

    x_min, x_max = x_unique.min(), x_unique.max()
    y_min, y_max = y_unique.min(), y_unique.max()

    grid_resolution = min(len(x_unique), len(y_unique), 100)
    xi = np.linspace(x_min, x_max, grid_resolution)
    yi = np.linspace(y_min, y_max, grid_resolution)
    X, Y = np.meshgrid(xi, yi)

    Z = griddata(
        terrain_points,
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

    for idx, robot_id in enumerate(robot_ids):
        trajectory = trajectories.get(robot_id, np.array([]))
        if len(trajectory) == 0:
            continue

        color = robot_colors[idx % len(robot_colors)]

        xs = trajectory[:, 0]
        ys = trajectory[:, 1]

        zs = [get_elevation_at(terrain_points, terrain_elevations, x, y) * elevation_scale + trajectory_z_offset
              for x, y in zip(xs, ys)]

        ax.plot(xs, ys, zs,
                color=color,
                linewidth=line_width,
                label=f'Robot {robot_id}')

        marker_xs = xs[::marker_interval]
        marker_ys = ys[::marker_interval]
        marker_zs = zs[::marker_interval]

        ax.scatter(marker_xs, marker_ys, marker_zs,
                   color=color,
                   s=marker_size,
                   marker='o',
                   edgecolors='black',
                   linewidths=0.5)

        if len(xs) > 0:
            ax.scatter([xs[0]], [ys[0]], [zs[0]],
                       color=color,
                       s=marker_size * 2,
                       marker='^',
                       edgecolors='black',
                       linewidths=1,
                       zorder=10)
            ax.scatter([xs[-1]], [ys[-1]], [zs[-1]],
                       color=color,
                       s=marker_size * 2,
                       marker='s',
                       edgecolors='black',
                       linewidths=1,
                       zorder=10)

        print(f'[INFO] Plotted {len(trajectory)} points for {robot_id}')


def generate_plot(data: dict) -> None:
    """Generate the 3D plot."""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    plot_terrain_surface(ax, data)
    plot_trajectories(ax, data)

    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_zlabel('Elevation [m]', fontsize=12)
    ax.set_title('3D Terrain with Rover Trajectories', fontsize=14)

    if any(len(t) > 0 for t in data['trajectories'].values()):
        ax.legend(loc='upper left')

    ax.view_init(elev=30, azim=45)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='View saved trajectory data')
    parser.add_argument('file', nargs='?', default='trajectory_data.npz',
                        help='Path to trajectory data file (default: trajectory_data.npz)')
    parser.add_argument('-z', '--z-offset', type=float, default=None,
                        help='Override trajectory Z offset (default: use saved value)')
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

    print('[INFO] Generating plot...')
    generate_plot(data)


if __name__ == '__main__':
    main()
