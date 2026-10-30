# Records a ROS 2 bag of the flocking run.
#
# Start it in its own terminal, after the simulator and before the flocking:
#     ros2 launch reynolds_flocking record_bag.launch.py bag_name:=my_test
#
# The bag is saved in <log_folder>/bags/my_test. log_folder is set in
# config/flocking_params.yaml. Stop it with Ctrl+C, so the bag is closed properly.

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    flocking_folder = get_package_share_directory('reynolds_flocking')
    parameters_path = os.path.join(flocking_folder, 'config', 'flocking_params.yaml')
    with open(parameters_path, 'r') as parameters_file:
        parameters = yaml.safe_load(parameters_file)
    log_folder = os.path.expanduser(parameters['logging']['ros__parameters']['log_folder'])

    bags_folder = os.path.join(log_folder, 'bags')
    os.makedirs(bags_folder, exist_ok=True)

    bag_name_argument = DeclareLaunchArgument(
        'bag_name',
        default_value='my_test',
        description='Name of the bag. It must not exist yet')

    # Commanded velocity, position, and the acceleration of every rule
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-o', PathJoinSubstitution([bags_folder, LaunchConfiguration('bag_name')]),
             '-e', '/robot_[0-9]+/(cmd_vel|odom|flocking/.*)'],
        output='screen')

    return LaunchDescription([
        bag_name_argument,
        record_bag,
    ])
