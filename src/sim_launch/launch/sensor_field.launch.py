import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get sensor field parameters
    sim_field_params = os.path.join(
        get_package_share_directory('sensor_field'),
        'config',
        'rf_disaster.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_field_config',
            default_value=sim_field_params,
            description='Path to sensor field configuration file'
        ),

        # Topography 2D sensor field node
        Node(
            package='sensor_field',
            executable='topography_2d',
            name='sample_2d',
            parameters=[LaunchConfiguration('sensor_field_config')],
        ),
    ])
