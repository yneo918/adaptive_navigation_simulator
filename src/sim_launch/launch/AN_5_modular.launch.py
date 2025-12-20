import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Field type configuration mapping
FIELD_CONFIG = {
    'paper': {
        'launch_file': 'paper_field.launch.py',
        'rviz_config': 'adaptive_navigation.rviz',
    },
    'disaster': {
        'launch_file': 'disaster_sensor_field.launch.py',
        'rviz_config': 'adaptive_navigation_dis.rviz',
    },
    'cesium': {
        'launch_file': 'ces_disaster_sensor_field.launch.py',
        'rviz_config': 'adaptive_navigation_ces.rviz',
    },
    'topography': {
        'launch_file': 'sensor_field.launch.py',
        'rviz_config': 'adaptive_navigation.rviz',
    },
}


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration based on field_type argument."""
    sim_launch_dir = get_package_share_directory('sim_launch')
    rover_desc_dir = get_package_share_directory('rover_description')

    # Get launch configurations
    field_type = LaunchConfiguration('field_type').perform(context)
    enable_controller = LaunchConfiguration('enable_controller')
    enable_sensor_field = LaunchConfiguration('enable_sensor_field')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_robots = LaunchConfiguration('enable_robots')

    # Validate field_type
    if field_type not in FIELD_CONFIG:
        raise ValueError(
            f'Invalid field_type: {field_type}. '
            f'Valid options: {", ".join(FIELD_CONFIG.keys())}'
        )

    config = FIELD_CONFIG[field_type]

    # Launch file paths
    controller_launch = os.path.join(sim_launch_dir, 'controller.launch.py')
    sensor_field_launch = os.path.join(sim_launch_dir, config['launch_file'])
    visualization_launch = os.path.join(sim_launch_dir, 'visualization.launch.py')
    robots_launch = os.path.join(sim_launch_dir, 'pioneer_5.launch.py')

    # RViz config path
    rviz_config = os.path.join(rover_desc_dir, 'rviz', config['rviz_config'])

    actions = []

    # Controller component
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            condition=IfCondition(enable_controller)
        )
    )

    # Sensor field component
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_field_launch),
            condition=IfCondition(enable_sensor_field)
        )
    )

    # Visualization component with field-specific rviz config
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            launch_arguments={'rviz_config': rviz_config}.items(),
            condition=IfCondition(enable_visualization)
        )
    )

    # Robots component
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robots_launch),
            condition=IfCondition(enable_robots)
        )
    )

    return actions


def generate_launch_description():
    """
    Modular launch file for adaptive navigation with 5 robots.

    Components:
    1. Controller nodes (cluster_controller + adaptive_nav)
    2. Sensor field (configurable via field_type)
    3. Visualization (GUI + RViz with field-specific config)
    4. Robots (5 pioneer robots)

    Launch arguments:
    - field_type: Sensor field type (paper, disaster, cesium, topography)
    - enable_controller: Enable controller nodes (default: true)
    - enable_sensor_field: Enable sensor field node (default: true)
    - enable_visualization: Enable GUI and RViz (default: true)
    - enable_robots: Enable robot simulation (default: true)

    Usage:
        ros2 launch sim_launch AN_5_modular.launch.py field_type:=paper
        ros2 launch sim_launch AN_5_modular.launch.py field_type:=disaster
        ros2 launch sim_launch AN_5_modular.launch.py field_type:=cesium
    """

    return LaunchDescription([
        # Field type selection
        DeclareLaunchArgument(
            'field_type',
            default_value='paper',
            description='Sensor field type: paper, disaster, cesium, topography'
        ),

        # Launch arguments for toggling components
        DeclareLaunchArgument(
            'enable_controller',
            default_value='true',
            description='Enable controller nodes (cluster_controller and adaptive_nav)'
        ),
        DeclareLaunchArgument(
            'enable_sensor_field',
            default_value='true',
            description='Enable sensor field node'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable visualization (GUI and RViz)'
        ),
        DeclareLaunchArgument(
            'enable_robots',
            default_value='true',
            description='Enable robot simulation (5 pioneer robots)'
        ),
        DeclareLaunchArgument(
            'use_hardware',
            default_value='false',
            description='Toggle hardware launch instead of simulation components'
        ),

        # Setup components based on field_type
        OpaqueFunction(function=launch_setup),
    ])
