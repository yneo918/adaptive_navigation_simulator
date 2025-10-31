import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Modular launch file for adaptive navigation with 5 robots.

    Components:
    1. Controller nodes (cluster_controller + adaptive_nav)
    2. Sensor field (topography_2d)
    3. Visualization (GUI + RViz)
    4. Robots (5 pioneer robots)

    Toggle components with launch arguments:
    - enable_controller: Enable controller nodes (default: true)
    - enable_sensor_field: Enable sensor field node (default: true)
    - enable_visualization: Enable GUI and RViz (default: true)
    - enable_robots: Enable robot simulation (default: true)
    """

    sim_launch_dir = get_package_share_directory('sim_launch')

    # Launch file paths
    controller_launch = os.path.join(sim_launch_dir, 'controller.launch.py')
    sensor_field_launch = os.path.join(sim_launch_dir, 'sensor_field.launch.py')
    visualization_launch = os.path.join(sim_launch_dir, 'visualization.launch.py')
    robots_launch = os.path.join(sim_launch_dir, 'pioneer_5.launch.py')

    return LaunchDescription([
        # Launch arguments for toggling components
        DeclareLaunchArgument(
            'enable_controller',
            default_value='true',
            description='Enable controller nodes (cluster_controller and adaptive_nav)'
        ),
        DeclareLaunchArgument(
            'enable_sensor_field',
            default_value='true',
            description='Enable sensor field node (topography_2d)'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable visualization (GUI and RViz)'
        ),
        DeclareLaunchArgument(
            'enable_robots',
            default_value='true',
            description='Enable robot simulation (5 pioneer robots)'
        ),
        DeclareLaunchArgument(
            'use_hardware',
            default_value='false',
            description='Toggle hardware launch instead of simulation components'
        ),

        # Controller component
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            condition=IfCondition(LaunchConfiguration('enable_controller'))
        ),

        # Sensor field component
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_field_launch),
            condition=IfCondition(LaunchConfiguration('enable_sensor_field'))
        ),

        # Visualization component
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            condition=IfCondition(LaunchConfiguration('enable_visualization'))
        ),

        # Robots component
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robots_launch),
            condition=IfCondition(LaunchConfiguration('enable_robots'))
        ),
    ])
