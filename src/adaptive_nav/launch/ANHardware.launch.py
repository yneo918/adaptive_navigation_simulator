import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    # Get the directory of this launch file
    package_name = 'cluster_node'
    pkg_share = get_package_share_directory(package_name)
    # Construct paths to the parameter files relative to the launch file directory
    cluster_file = os.path.join(pkg_share, 'config', '3cluster_velocity.yaml')
    display_launch_file = os.path.join(get_package_share_directory('rover_description'), 'launch', 'display.launch.py')
    pioneer_launch_file = os.path.join(get_package_share_directory('sim_launch'), 'pioneer_with_hw_desired.launch.py')
    custom_rviz_config = os.path.join(get_package_share_directory('rover_description'), 'rviz/clusterp1-p3withhw.rviz')

    # Check if parameter files exist
    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    return LaunchDescription([
        Node(
            package="controller",
            executable="cluster_controller",
            name="cluster_feedback", #name must match yaml file
            parameters=[cluster_file], 
        ),
        Node(
            package="adaptive_nav",
            executable="adaptive_nav",
            name="cluster_feedback", #name must match yaml file
            parameters=[cluster_file],
        ),
        Node(
            package="virtual_joy",
            executable="virtual_joy",
        ),
        Node(
        package="teleop_core",
        executable="cmd_demux",
        parameters=["pioneer_base/teleop_core/config/demux.yaml"],
        ),
        Node(
        package="teleop_core",
        executable="joywithgui3",
        parameters=["pioneer_base/teleop_core/config/joy-assign.yaml"],
        ),   
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={'rvizconfig': custom_rviz_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file)
        )
    ])