"""
Launch file for 3D trajectory plotter.

Usage:
    ros2 launch sim_launch trajectory_plotter.launch.py

    # With custom parameters
    ros2 launch sim_launch trajectory_plotter.launch.py num_robots:=3 output_file:=my_plot.png
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='5',
            description='Number of robots to track'
        ),
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='/sim',
            description='Prefix for robot topics'
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='trajectory_3d.png',
            description='Output filename for the plot'
        ),
        DeclareLaunchArgument(
            'dpi',
            default_value='150',
            description='DPI for output image'
        ),
        DeclareLaunchArgument(
            'elevation_scale',
            default_value='1.0',
            description='Scale factor for terrain elevation'
        ),
        DeclareLaunchArgument(
            'trajectory_sample_interval',
            default_value='0.5',
            description='Time interval (seconds) between trajectory samples'
        ),
        DeclareLaunchArgument(
            'marker_interval',
            default_value='10',
            description='Plot a marker every N trajectory points'
        ),

        Node(
            package='sensor_field',
            executable='trajectory_plotter_3d',
            name='trajectory_plotter_3d',
            output='screen',
            parameters=[{
                'num_robots': LaunchConfiguration('num_robots'),
                'robot_prefix': LaunchConfiguration('robot_prefix'),
                'robot_ids': ['p0', 'p1', 'p2', 'p3', 'p4'],
                'output_file': LaunchConfiguration('output_file'),
                'dpi': LaunchConfiguration('dpi'),
                'elevation_scale': LaunchConfiguration('elevation_scale'),
                'trajectory_sample_interval': LaunchConfiguration('trajectory_sample_interval'),
                'marker_interval': LaunchConfiguration('marker_interval'),
                'terrain_alpha': 0.7,
                'line_width': 2.0,
                'marker_size': 30,
            }],
        ),
    ])
