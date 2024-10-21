# exploration.launch.yaml

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import Shutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('node_name', default_value='exploration_random',
                          description='exploration node name'),
    DeclareLaunchArgument('algorithm_variant', default_value=''),
    DeclareLaunchArgument('world', default_value='small',
                          choices=['small', 'large'],
                          description='exploration world'),
    DeclareLaunchArgument('results_file', default_value='~/exploration_results.csv',
                          description='path of the results file'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false']),
]

def generate_launch_description():

    # Directories
    pkg_upc_mrn = get_package_share_directory(
        'upc_mrn')

    # launch path
    frontiers_launch = PathJoinSubstitution(
        [pkg_upc_mrn, 'launch', 'frontiers.launch.yaml'])
        
    # frontiers
    frontiers = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(frontiers_launch)
    )

    # exploration
    exploration = Node(
        package="upc_mrn",
        executable=LaunchConfiguration('node_name'),
        name=LaunchConfiguration('node_name'),
        output="screen",
        on_exit=Shutdown(),
        parameters=[{"world": LaunchConfiguration('world'),
                     "algorithm_variant": LaunchConfiguration('algorithm_variant'),
                     "results_file": LaunchConfiguration('results_file'),
                     # ADD YOUR PARAMETERS HERE:
                     # "param_name": 3.5,
                     }]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(frontiers)
    ld.add_action(exploration)
    return ld
