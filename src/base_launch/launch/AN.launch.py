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
    cluster_file = os.path.join(pkg_share, 'config', '3cluster_velocity.yaml')

    display_launch_file = os.path.join(
        get_package_share_directory('rover_description'),
        'launch',
        'display.launch.py',
    )
    sim_rviz_config = os.path.join(
        get_package_share_directory('rover_description'),
        'rviz/clusterp1-p3withdesired.rviz',
    )
    hardware_rviz_config = os.path.join(
        get_package_share_directory('rover_description'),
        'rviz/clusterp1-p3withhw.rviz',
    )
    hardware_pioneer_launch = os.path.join(
        get_package_share_directory('sim_launch'),
        'pioneer_with_hw_desired.launch.py',
    )
    teleop_config_dir = os.path.join(
        get_package_share_directory('teleop_core'),
        'config',
    )
    demux_config = os.path.join(teleop_config_dir, 'demux.yaml')
    joy_assign_config = os.path.join(teleop_config_dir, 'joy-assign.yaml')

    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")
    if not os.path.isfile(demux_config):
        raise FileNotFoundError(f"Parameter file not found: {demux_config}")
    if not os.path.isfile(joy_assign_config):
        raise FileNotFoundError(f"Parameter file not found: {joy_assign_config}")

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
            package='rf_sim',
            executable='rf_field',
            condition=UnlessCondition(use_hardware),
        ),
        Node(
            package='virtual_joy',
            executable='virtual_joy',
        ),
        Node(
            package='teleop_core',
            executable='cmd_demux',
            parameters=[demux_config],
        ),
        Node(
            package='teleop_core',
            executable='joywithgui3',
            parameters=[joy_assign_config],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={'rvizconfig': sim_rviz_config}.items(),
            condition=UnlessCondition(use_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={'rvizconfig': hardware_rviz_config}.items(),
            condition=IfCondition(use_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_pioneer_launch),
            condition=IfCondition(use_hardware),
        ),
    ])
