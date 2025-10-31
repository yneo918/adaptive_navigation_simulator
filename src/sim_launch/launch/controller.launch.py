import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get cluster configuration file
    package_name = 'cluster_node'
    pkg_share = get_package_share_directory(package_name)
    cluster_file = os.path.join(pkg_share, 'config', '5cluster_velocity.yaml')

    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'cluster_config',
            default_value=cluster_file,
            description='Path to cluster configuration file'
        ),

        # Cluster controller node
        Node(
            package='controller',
            executable='cluster_controller',
            name='cluster_feedback',
            parameters=[LaunchConfiguration('cluster_config')],
        ),

        # Adaptive navigation node
        Node(
            package='adaptive_nav',
            executable='adaptive_nav',
            name='cluster_feedback',
            parameters=[LaunchConfiguration('cluster_config')],
        ),
    ])
