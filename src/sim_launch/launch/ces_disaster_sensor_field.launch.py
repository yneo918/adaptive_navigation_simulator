import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.conditions


def generate_launch_description():
    # Get sensor field parameters
    sim_field_params = os.path.join(
        get_package_share_directory('sensor_field'),
        'config',
        'cesium_field.yaml',
    )

    # Get ces config with contour visualizer parameters
    ces_config = os.path.join(
        get_package_share_directory('sensor_field'),
        'config',
        'ces.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_field_config',
            default_value=sim_field_params,
            description='Path to sensor field configuration file'
        ),
        DeclareLaunchArgument(
            'enable_contour_visualizer',
            default_value='true',
            description='Enable contour line visualization in RViz'
        ),

        # Topography 2D sensor field node
        Node(
            package='sensor_field',
            executable='cesium_sensor_field',
            name='cesium_sensor_field',
            parameters=[LaunchConfiguration('sensor_field_config')],
        ),

        # Contour visualizer node
        Node(
            package='sensor_field',
            executable='contour_visualizer',
            name='contour_visualizer',
            parameters=[ces_config],
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('enable_contour_visualizer')
            ),
        ),
    ])
