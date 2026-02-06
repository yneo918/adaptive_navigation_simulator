from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'rover_description'
    pioneer_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'pioneer.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'time_scale',
            default_value='1.0',
            description='Simulation time scale (1.0=real-time, 2.0=2x speed)'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p1',
                'x': '20.0',
                'y': '170.0',
                't': '1.57',
                'desired': 'desired',
                'time_scale': LaunchConfiguration('time_scale'),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p2',
                'x': '20.0',
                'y': '170.0',
                't': '1.57',
                'desired': 'desired',
                'time_scale': LaunchConfiguration('time_scale'),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p3',
                'x': '20.0',
                'y': '170.0',
                't': '1.57',
                'desired': 'desired',
                'time_scale': LaunchConfiguration('time_scale'),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p4',
                'x': '20.0',
                'y': '170.0',
                't': '1.57',
                'desired': 'desired',
                'time_scale': LaunchConfiguration('time_scale'),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p5',
                'x': '20.0',
                'y': '170.0',
                't': '1.57',
                'desired': 'desired',
                'time_scale': LaunchConfiguration('time_scale'),
            }.items()
        )
    ])
