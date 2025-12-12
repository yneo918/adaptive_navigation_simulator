import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node

COLORS = {
    'p1': {'r': 1.0, 'g': 0.0, 'b': 0.0},
    'p2': {'r': 0.0, 'g': 1.0, 'b': 0.0},
    'p3': {'r': 0.0, 'g': 0.0, 'b': 1.0},
    'p4': {'r': 1.0, 'g': 1.0, 'b': 0.0},
    'p5': {'r': 1.0, 'g': 0.0, 'b': 1.0},
    'p6': {'r': 0.0, 'g': 1.0, 'b': 1.0},
}


def _load_urdf_with_color(pkg_share: str, robot_id: str, alpha: float = 1.0) -> str:
    """Load URDF template and substitute color values"""
    template_path = os.path.join(pkg_share, 'urdf/pioneer_template.urdf')
    color = COLORS.get(robot_id, {'r': 0.5, 'g': 0.5, 'b': 0.5})

    with open(template_path, 'r') as f:
        urdf = f.read()

    return urdf.format(r=color['r'], g=color['g'], b=color['b'], a=alpha)


def generate_launch_description():
    # Get package directory
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='rover_description'
    ).find('rover_description')

    # RViz config file path
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/cluster3withhw.rviz')

    # Robot configurations
    robots = [
        {'id': 'p2', 'x': 0.0, 'y': 0.0, 't': 0.0},
        {'id': 'p3', 'x': 5.0, 'y': 5.0, 't': 0.0},
        {'id': 'p4', 'x': 5.0, 'y': -5.0, 't': 0.0},
    ]

    nodes = [
        # Specify RViz config file path
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file"
        ),
        # Enable time for Gazebo and simulation
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time"
        ),
        # Launch RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
        Node(
            package="register_service",
            executable="register_service",
            name="robot_register_server",
            output="screen",
        ),
    ]

    # Add nodes for each robot
    for robot in robots:
        robot_id = robot['id']
        robot_description = _load_urdf_with_color(pkg_share, robot_id)

        # Robot state publisher
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                namespace=robot_id,
                output="screen",
                parameters=[{
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_prefix": f"{robot_id}/",
                }]
            )
        )

        # TF broadcaster
        nodes.append(
            Node(
                package="fake_rover_state_controller",
                executable="tf_broadcaster",
                name="tf_broadcaster",
                namespace=robot_id,
                output="screen",
                parameters=[{
                    'robot_id': robot_id,
                    'with_desired': False,
                }]
            )
        )

        # Fake rover
        nodes.append(
            Node(
                package="fake_rover_state_controller",
                executable="fake_rover",
                name="fake_rover",
                namespace=robot_id,
                output="screen",
                parameters=[{
                    'robot_id': robot_id,
                    'x': robot['x'],
                    'y': robot['y'],
                    't': robot['t'],
                    'prefix': '/sim'
                }]
            )
        )

    return LaunchDescription(nodes)
