import os
import launch
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

COLORS = {
    'p1': [1.0, 0.0, 0.0],
    'p2': [0.0, 1.0, 0.0],
    'p3': [0.0, 0.0, 1.0],
    'p4': [1.0, 1.0, 0.0],
    'p5': [1.0, 0.0, 1.0],
    'p6': [0.0, 1.0, 1.0],
}

def launch_setup(context, *args, **kwargs):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    robot_id = LaunchConfiguration("robot_id").perform(context)
    hw = LaunchConfiguration("hw").perform(context)
    desired = LaunchConfiguration("desired").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    t = float(LaunchConfiguration("t").perform(context))

    # Use pre-compiled URDF files instead of xacro
    urdf_file = os.path.join(pkg_share, f'urdf/pioneer_{robot_id}.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    urdf_desired_file = os.path.join(pkg_share, f'urdf/pioneer_{robot_id}_desired.urdf')
    with open(urdf_desired_file, 'r') as f:
        robot_description_desired = f.read()


    main_nodes = [
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
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}/world"],
        ),
        Node(
            package='fake_rover_state_controller',
            executable='sim_rover',
            name='sim_rover',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'x': x,
                'y': y,
                't': t,
                'prefix': ''
            }]
        ),
        Node(
            package='fake_rover_state_controller',
            executable='fake_sensor',
            name='fake_sensor',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'prefix': '',
                'sensor_msg_name': 'sensor',
                'sensor_service_name': 'get_sensor'
            }]
        ),
        Node(
            package='fake_rover_state_controller',
            executable='jointstate_publisher',
            name='jointstate_publisher',
            output='screen',
            parameters=[{
                'robot_id': robot_id
            }]
        ),
    ]
    if desired == 'desired':
        main_nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                namespace=f"{robot_id}desired",
                output="screen",
                parameters=[{
                    "robot_description": robot_description_desired,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_prefix": f"{robot_id}desired/",
                }]
            )
        )
        main_nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}desired/world"],
            ),
        )
    return main_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="p1", description="Robot ID"),
        DeclareLaunchArgument("x", default_value="0.0", description="X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Y position"),
        DeclareLaunchArgument("t", default_value="0.0", description="Theta"),
        DeclareLaunchArgument("a", default_value="1.0", description="Transparency"),
        DeclareLaunchArgument("hwa", default_value="0.5", description="Transparency of hw"),
        DeclareLaunchArgument("hw", default_value="", description="Transparency"),
        DeclareLaunchArgument("desired", default_value="", description="Transparency"),
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        OpaqueFunction(function=launch_setup),
    ])
