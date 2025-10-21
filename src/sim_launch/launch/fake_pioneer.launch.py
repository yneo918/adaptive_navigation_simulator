from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    return LaunchDescription([
        Node(
            package='fake_rover_state_controller',
            executable='fake_rover',
            name='fake_rover',
            output='screen',
            parameters=[{
                'robot_id': "p1",
                'x': -6.0, 
                'y': 5.0,
                't': 0.0
            }]
        ),
        Node(
            package='fake_rover_state_controller',
            executable='fake_rover',
            name='fake_rover',
            output='screen',
            parameters=[{
                'robot_id': "p2",
                'x': -3.0, 
                'y': 5.0,
                't': 0.0
            }]
        ),
        Node(
            package='fake_rover_state_controller',
            executable='fake_rover',
            name='fake_rover',
            output='screen',
            parameters=[{
                'robot_id': "p3",
                'x': 0.0, 
                'y': 5.0,
                't': 0.0
            }]
        ),
        Node(
            package='fake_rover_state_controller',
            executable='fake_rover',
            name='fake_rover',
            output='screen',
            parameters=[{
                'robot_id': "p4",
                'x': 3.0, 
                'y': 5.0,
                't': 0.0
            }]
        ),
        Node(
            package='fake_rover_state_controller',
            executable='fake_rover',
            name='fake_rover',
            output='screen',
            parameters=[{
                'robot_id': "p5",
                'x': 6.0, 
                'y': 5.0,
                't': 0.0
            }]
        )
    ])
