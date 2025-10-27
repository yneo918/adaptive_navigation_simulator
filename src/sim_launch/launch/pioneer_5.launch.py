from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'rover_description'
    pioneer_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'pioneer.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p1',
                'x': '-30.0', 
                'y': '-10.0',
                't': '1.57',
                'desired': 'desired',
                'a': '0.2'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p2',
                'x': '-30.0', 
                'y': '-7.0',
                't': '1.57',
                'desired': 'desired',
                'a': '0.2'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p3',
                'x': '-30.0', 
                'y': '-13.0', 
                't': '1.57',
                'desired': 'desired',
                'a': '0.2'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p4',
                'x': '-30.0', 
                'y': '-4.0',
                't': '1.57',
                'desired': 'desired',
                'a': '0.2'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p5',
                'x': '-30.0', 
                'y': '-16.0',
                't': '1.57',
                'desired': 'desired',
                'a': '0.2'
            }.items()
        )
    ])
