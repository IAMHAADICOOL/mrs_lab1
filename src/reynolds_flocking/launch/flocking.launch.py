# Starts the flocking node.
#
# Start the simulator first (in another terminal):
#     ros2 launch sphero_stage start.launch.py
# Then:
#     ros2 launch reynolds_flocking flocking.launch.py
#
# Optional arguments:
#     params_file:=/path/to/my_params.yaml   use another parameter file

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Read the number of robots from the simulator settings, so the
    # flocking node always controls exactly the robots that exist.
    sphero_stage_folder = get_package_share_directory('sphero_stage')
    simulator_settings_path = os.path.join(sphero_stage_folder, 'launch', 'launch_params.yaml')
    with open(simulator_settings_path, 'r') as settings_file:
        simulator_settings = yaml.safe_load(settings_file)
    num_robots = int(simulator_settings['num_of_robots'])

    flocking_folder = get_package_share_directory('reynolds_flocking')
    default_params_file = os.path.join(flocking_folder, 'config', 'flocking_params.yaml')

    params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='YAML file with the flocking parameters')

    flocking_node = Node(
        package='reynolds_flocking',
        executable='flocking_node',
        name='reynolds_flocking',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'num_robots': num_robots},
        ],
    )

    return LaunchDescription([
        params_file_argument,
        flocking_node,
    ])
