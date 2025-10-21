from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


package_name = 'adaptive_navigation_utilities'

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('record_all_topics', default_value='True'),
        DeclareLaunchArgument('timeout', default_value='60.0'),
        DeclareLaunchArgument('period', default_value='0.05'),

        Node(
            package=package_name,
            executable='live_data_to_csv',
            name='live_data_to_csv',
            parameters=[
                config_file,
                {
                    'record_all_topics': LaunchConfiguration('record_all_topics'),
                    'timeout': LaunchConfiguration('timeout'),
                    'period': LaunchConfiguration('period'),
                }
            ]
        )
    ])
