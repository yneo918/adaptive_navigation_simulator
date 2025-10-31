import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get display launch file and RViz config
    display_launch_file = os.path.join(
        get_package_share_directory('rover_description'),
        'launch',
        'display.launch.py',
    )

    default_rviz_config = os.path.join(
        get_package_share_directory('rover_description'),
        'rviz/adaptive_navigation.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz configuration file'
        ),

        DeclareLaunchArgument(
            'enable_gui',
            default_value='true',
            description='Enable GUI for adaptive navigation'
        ),

        # GUI node
        Node(
            package='gui_package',
            executable='gui_adaptive_navigation',
        ),

        # RViz visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={'rvizconfig': LaunchConfiguration('rviz_config')}.items(),
        ),
    ])
