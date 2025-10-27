import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'cluster_node'
    pkg_share = get_package_share_directory(package_name)
    cluster_file = os.path.join(pkg_share, 'config', '5cluster_velocity.yaml')
    sim_field_params = os.path.join(
        get_package_share_directory('sensor_field'),
        'config',
        'rf_disaster.yaml',
    )

    display_launch_file = os.path.join(
        get_package_share_directory('rover_description'),
        'launch',
        'display.launch.py',
    )
    rviz_config = os.path.join(
        get_package_share_directory('rover_description'),
        'rviz/adaptive_navigation.rviz',
    )
    pioneer_launch = os.path.join(
        get_package_share_directory('sim_launch'),
        'pioneer_5.launch.py',
    )
    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    use_hardware = LaunchConfiguration('use_hardware')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_hardware',
            default_value='false',
            description='Toggle hardware launch instead of simulation components.',
        ),
        Node(
            package='controller',
            executable='cluster_controller',
            name='cluster_feedback',
            parameters=[cluster_file],
        ),
        Node(
            package='adaptive_nav',
            executable='adaptive_nav',
            name='cluster_feedback',
            parameters=[cluster_file],
        ),
        Node(
            package='sensor_field',
            executable='sample_2d',
            name='sample_2d',
            parameters=[sim_field_params],
        ),
        Node(
            package='gui_package',
            executable='gui_adaptive_navigation',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={'rvizconfig': rviz_config}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch),
        ),
    ])
