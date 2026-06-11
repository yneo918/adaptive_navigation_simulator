"""Headless adaptive navigation launch (no RViz, no PyQt GUI).

Persistent stack for batch experiments. The trajectory_plotter_3d is NOT
included here; the orchestrator spawns it per run and SIGINTs it when the
run terminates so that one NPZ is produced per experiment.
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cluster_share = get_package_share_directory('cluster_node')
    cluster_file = os.path.join(cluster_share, 'config', '5cluster_velocity.yaml')

    sensor_field_share = get_package_share_directory('sensor_field')
    cesium_field_yaml = os.path.join(sensor_field_share, 'config', 'cesium_field.yaml')

    sim_launch_share = get_package_share_directory('sim_launch')
    pioneer_launch = os.path.join(sim_launch_share, 'pioneer_5.launch.py')

    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f'Parameter file not found: {cluster_file}')

    return LaunchDescription([
        DeclareLaunchArgument(
            'time_scale',
            default_value='1.0',
            description='Simulation time scale'
        ),

        # Cluster controller.
        # use_sim_time: timers follow the scaled /clock from clock_publisher
        # (launched with the first robot), so the control loop accelerates
        # with the simulation. time_scale stays 1.0 here on purpose: the
        # timer period is interpreted in SIM time, so dividing it by
        # time_scale again (as the GUI controller.launch.py does) would
        # inflate the control rate per simulated second and change the
        # controller's behaviour across acceleration settings.
        Node(
            package='controller',
            executable='cluster_controller',
            name='cluster_feedback',
            parameters=[cluster_file,
                        {'use_sim_time': True, 'time_scale': 1.0}],
            output='screen',
        ),

        # Adaptive navigation (same clocking rationale as the controller)
        Node(
            package='adaptive_nav',
            executable='adaptive_nav',
            name='cluster_feedback',
            parameters=[cluster_file,
                        {'use_sim_time': True, 'time_scale': 1.0}],
            output='screen',
        ),

        # Cesium sensor field
        Node(
            package='sensor_field',
            executable='cesium_sensor_field',
            name='cesium_sensor_field',
            parameters=[cesium_field_yaml],
            output='screen',
        ),

        # Headless GUI replacement
        Node(
            package='auto_runner',
            executable='headless_an_controller',
            name='headless_an_controller',
            output='screen',
        ),

        # 5 pioneer robots
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch),
            launch_arguments={
                'time_scale': LaunchConfiguration('time_scale'),
            }.items(),
        ),
    ])
