import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
DESIRED_ALPHA = 0.2


def _load_urdf_with_color(pkg_share: str, robot_id: str, alpha: float) -> str:
    """Load URDF template and substitute color values"""
    template_path = os.path.join(pkg_share, 'urdf/pioneer_template.urdf')
    color = COLORS.get(robot_id, {'r': 0.5, 'g': 0.5, 'b': 0.5})

    with open(template_path, 'r') as f:
        urdf = f.read()

    return urdf.format(r=color['r'], g=color['g'], b=color['b'], a=alpha)


def launch_setup(context, *args, **kwargs):
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='rover_description'
    ).find('rover_description')

    robot_id = LaunchConfiguration("robot_id").perform(context)
    desired = LaunchConfiguration("desired").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    t = float(LaunchConfiguration("t").perform(context))
    alpha = float(LaunchConfiguration("a").perform(context))
    time_scale = float(LaunchConfiguration("time_scale").perform(context))

    with_desired = desired == 'desired'
    is_first_robot = (robot_id == 'p1')

    # Load URDF with color substitution
    robot_description = _load_urdf_with_color(pkg_share, robot_id, alpha)

    main_nodes = [
        # Robot state publisher for visualization (URDF -> RViz)
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
        # TF broadcaster (world -> base_link)
        Node(
            package='fake_rover_state_controller',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'with_desired': with_desired,
                'use_sim_time': LaunchConfiguration("use_sim_time"),
            }]
        ),
        # Simulation rover (differential drive)
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
                'prefix': '',
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'time_scale': time_scale,
            }]
        ),
        # Fake sensor
        Node(
            package='fake_rover_state_controller',
            executable='fake_sensor',
            name='fake_sensor',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'prefix': '',
                'sensor_msg_name': 'sensor',
                'sensor_service_name': 'get_sensor',
                'use_sim_time': LaunchConfiguration("use_sim_time"),
            }]
        ),
    ]

    # Add desired robot state publisher if needed
    if with_desired:
        robot_description_desired = _load_urdf_with_color(
            pkg_share, robot_id, DESIRED_ALPHA
        )
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

    # Add clock publisher (only for first robot to avoid conflicts)
    if is_first_robot:
        # Scale clock publish rate with time_scale, capped at 10000Hz (supports time_scale up to 1000)
        clock_publish_rate = min(100.0 * time_scale, 10000.0)
        main_nodes.append(
            Node(
                package='fake_rover_state_controller',
                executable='clock_publisher',
                name='clock_publisher',
                output='screen',
                parameters=[{
                    'time_scale': time_scale,
                    'publish_rate_hz': clock_publish_rate,
                }]
            )
        )

    return main_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="p1", description="Robot ID"),
        DeclareLaunchArgument("x", default_value="0.0", description="X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Y position"),
        DeclareLaunchArgument("t", default_value="0.0", description="Theta"),
        DeclareLaunchArgument("a", default_value="1.0", description="Alpha (transparency)"),
        DeclareLaunchArgument("desired", default_value="", description="Enable desired visualization"),
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),
        DeclareLaunchArgument("time_scale", default_value="1.0", description="Simulation time scale (1.0=real-time, 2.0=2x speed)"),

        OpaqueFunction(function=launch_setup),
    ])
