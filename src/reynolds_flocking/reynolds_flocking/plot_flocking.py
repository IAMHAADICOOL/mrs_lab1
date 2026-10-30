# Draws graphs from a ROS 2 bag recorded during a flocking run.
#
# Usage:
#     ros2 run reynolds_flocking plot_flocking --bag_name my_test
#     ros2 run reynolds_flocking plot_flocking --bag_name my_test --robot 3
#     ros2 run reynolds_flocking plot_flocking --bag_name my_test --plot_folder ~/my_plots
#     ros2 run reynolds_flocking plot_flocking --bag_name my_test --show
#     ros2 run reynolds_flocking plot_flocking --bag_folder ~/somewhere/my_test
#
# --bag_name looks in <log_folder>/bags. log_folder is set in flocking_params.yaml.
#
# It saves these pictures in <log_folder>/plots/<bag name>, or in 'plot_folder'
# if you give one:
#     velocities_all_robots.png  commanded vs. actual vx and vy of every robot
#     velocity_robot_X.png       commanded vs. actual vx and vy of one robot
#     accelerations_robot_X.png  separation, alignment, cohesion, obstacle and total
#
# Actual velocity:
#     Stage repeats the commanded velocity in the twist part of /robot_X/odom,
#     even when a robot is blocked and not moving. So we do not use that. We
#     compute the actual velocity from how the position in /robot_X/odom changes.

import argparse
import math
import os

import matplotlib
import rosbag2_py
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# Actual velocity is measured over this many odom messages. Odom arrives
# 100 times per second, so 20 messages is 0.2 seconds.
ACTUAL_VELOCITY_MESSAGES = 20


def read_log_folder():
    """Read log_folder from flocking_params.yaml."""
    package_folder = get_package_share_directory('reynolds_flocking')
    parameters_path = os.path.join(package_folder, 'config', 'flocking_params.yaml')
    with open(parameters_path, 'r') as parameters_file:
        parameters = yaml.safe_load(parameters_file)
    return os.path.expanduser(parameters['logging']['ros__parameters']['log_folder'])


def create_empty_robot():
    robot = {
        'yaw': 0.0,
        'command_time': [], 'command_vx': [], 'command_vy': [],
        'odom_time': [], 'odom_x': [], 'odom_y': [],
        'actual_time': [], 'actual_vx': [], 'actual_vy': [],
    }
    for name in ['separation', 'alignment', 'cohesion', 'obstacle', 'total_acceleration']:
        robot[name + '_time'] = []
        robot[name + '_x'] = []
        robot[name + '_y'] = []
    return robot


def yaw_from_odom(message):
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(bag_folder):
    """Returns a dictionary: robot_id -> all the data of that robot."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_folder, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))

    message_classes = {}
    for topic in reader.get_all_topics_and_types():
        message_classes[topic.name] = get_message(topic.type)

    robots = {}
    while reader.has_next():
        topic_name, raw_data, time_nanoseconds = reader.read_next()
        seconds = time_nanoseconds / 1000000000.0

        # A topic name looks like /robot_3/cmd_vel or /robot_3/flocking/alignment.
        # Splitting at '/' gives ['', 'robot_3', 'flocking', 'alignment']
        parts = topic_name.split('/')
        if len(parts) < 3 or not parts[1].startswith('robot_'):
            continue
        robot_id = int(parts[1].replace('robot_', ''))
        if robot_id not in robots:
            robots[robot_id] = create_empty_robot()
        robot = robots[robot_id]

        message = deserialize_message(raw_data, message_classes[topic_name])

        if parts[2] == 'odom':
            robot['yaw'] = yaw_from_odom(message)
            robot['odom_time'].append(seconds)
            robot['odom_x'].append(message.pose.pose.position.x)
            robot['odom_y'].append(message.pose.pose.position.y)

        elif parts[2] == 'cmd_vel':
            # cmd_vel is in the robot's own frame, so rotate it to the world frame
            yaw = robot['yaw']
            robot['command_time'].append(seconds)
            robot['command_vx'].append(math.cos(yaw) * message.linear.x - math.sin(yaw) * message.linear.y)
            robot['command_vy'].append(math.sin(yaw) * message.linear.x + math.cos(yaw) * message.linear.y)

        elif parts[2] == 'flocking':
            name = parts[3]
            robot[name + '_time'].append(seconds)
            robot[name + '_x'].append(message.vector.x)
            robot[name + '_y'].append(message.vector.y)

    return robots


def compute_actual_velocity(robot):
    """Actual velocity = change of position / change of time.

    Compares each position with the position ACTUAL_VELOCITY_MESSAGES later.
    The result is drawn at the time in the middle of those two positions.
    """
    steps = ACTUAL_VELOCITY_MESSAGES
    for k in range(len(robot['odom_time']) - steps):
        time_difference = robot['odom_time'][k + steps] - robot['odom_time'][k]
        if time_difference <= 0.0:
            continue
        x_difference = robot['odom_x'][k + steps] - robot['odom_x'][k]
        y_difference = robot['odom_y'][k + steps] - robot['odom_y'][k]
        robot['actual_time'].append(robot['odom_time'][k + steps // 2])
        robot['actual_vx'].append(x_difference / time_difference)
        robot['actual_vy'].append(y_difference / time_difference)


def remove_data_after_last_command(robots):
    """When the flocking node stops, the robots keep moving for a few seconds
    with no control, until Stage stops them. That is not part of the run."""
    end_time = 0.0
    for robot in robots.values():
        if len(robot['command_time']) > 0 and robot['command_time'][-1] > end_time:
            end_time = robot['command_time'][-1]

    for robot in robots.values():
        kept_time = []
        kept_vx = []
        kept_vy = []
        for k in range(len(robot['actual_time'])):
            if robot['actual_time'][k] <= end_time:
                kept_time.append(robot['actual_time'][k])
                kept_vx.append(robot['actual_vx'][k])
                kept_vy.append(robot['actual_vy'][k])
        robot['actual_time'] = kept_time
        robot['actual_vx'] = kept_vx
        robot['actual_vy'] = kept_vy


def shift_all_times(robots):
    """Make time 0 the moment of the first velocity command."""
    start_time = None
    for robot in robots.values():
        if len(robot['command_time']) == 0:
            continue
        if start_time is None or robot['command_time'][0] < start_time:
            start_time = robot['command_time'][0]

    for robot in robots.values():
        for name in robot:
            if name.endswith('_time'):
                shifted_times = []
                for value in robot[name]:
                    shifted_times.append(value - start_time)
                robot[name] = shifted_times


def length(x, y):
    return math.sqrt(x * x + y * y)


def plot_all_velocities(robots, output_folder, pyplot):
    """One row per robot: vx on the left and vy on the right."""
    robot_ids = sorted(robots.keys())
    rows = len(robot_ids)

    figure, axes = pyplot.subplots(rows, 2, figsize=(18, 2.6 * rows), sharex=True, squeeze=False)
    for row in range(rows):
        robot = robots[robot_ids[row]]
        name = 'robot_' + str(robot_ids[row])

        axes[row][0].plot(robot['command_time'], robot['command_vx'], label='commanded')
        axes[row][0].plot(robot['actual_time'], robot['actual_vx'], '--', label='actual')
        axes[row][0].set_title(name + ': vx', fontsize=11)
        axes[row][0].set_ylabel('vx [m/s]')

        axes[row][1].plot(robot['command_time'], robot['command_vy'], label='commanded')
        axes[row][1].plot(robot['actual_time'], robot['actual_vy'], '--', label='actual')
        axes[row][1].set_title(name + ': vy', fontsize=11)
        axes[row][1].set_ylabel('vy [m/s]')

        for axis in axes[row]:
            axis.set_xlim(left=0.0)
            axis.grid(True)

    axes[0][0].legend(fontsize=10)
    axes[rows - 1][0].set_xlabel('time [s]')
    axes[rows - 1][1].set_xlabel('time [s]')
    figure.suptitle('Commanded vs. actual velocity of every robot', fontsize=14)
    figure.tight_layout()
    path = os.path.join(output_folder, 'velocities_all_robots.png')
    # High resolution, so the picture stays sharp when you zoom in
    figure.savefig(path, dpi=250)
    return path


def plot_one_robot_velocity(robots, robot_id, output_folder, pyplot):
    robot = robots[robot_id]
    figure, axes = pyplot.subplots(2, 1, figsize=(7, 4.5), sharex=True)

    axes[0].plot(robot['command_time'], robot['command_vx'], label='commanded vx')
    axes[0].plot(robot['actual_time'], robot['actual_vx'], '--', label='actual vx')
    axes[0].set_ylabel('vx [m/s]')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(robot['command_time'], robot['command_vy'], label='commanded vy')
    axes[1].plot(robot['actual_time'], robot['actual_vy'], '--', label='actual vy')
    axes[1].set_ylabel('vy [m/s]')
    axes[1].set_xlabel('time [s]')
    axes[1].set_xlim(left=0.0)
    axes[1].legend()
    axes[1].grid(True)

    figure.suptitle('robot_' + str(robot_id) + ': commanded vs. actual velocity')
    figure.tight_layout()
    path = os.path.join(output_folder, 'velocity_robot_' + str(robot_id) + '.png')
    figure.savefig(path, dpi=120)
    return path


def plot_accelerations(robots, robot_id, output_folder, pyplot):
    robot = robots[robot_id]
    names = ['separation', 'alignment', 'cohesion', 'obstacle', 'total_acceleration']

    figure, axes = pyplot.subplots(3, 1, figsize=(7, 5.5), sharex=True)
    for name in names:
        times = robot[name + '_time']
        x_values = robot[name + '_x']
        y_values = robot[name + '_y']
        sizes = []
        for k in range(len(x_values)):
            sizes.append(length(x_values[k], y_values[k]))

        if name == 'total_acceleration':
            label = 'total'
            line_width = 2.0
            line_color = 'black'
        else:
            label = name
            line_width = 1.0
            line_color = None

        axes[0].plot(times, x_values, label=label, linewidth=line_width, color=line_color)
        axes[1].plot(times, y_values, label=label, linewidth=line_width, color=line_color)
        axes[2].plot(times, sizes, label=label, linewidth=line_width, color=line_color)

    axes[0].set_ylabel('a_x [m/s²]')
    axes[1].set_ylabel('a_y [m/s²]')
    axes[2].set_ylabel('|a| [m/s²]')
    axes[2].set_xlabel('time [s]')
    axes[2].set_xlim(left=0.0)
    for axis in axes:
        axis.grid(True)
    axes[0].legend(loc='upper right', fontsize=8)
    figure.suptitle('robot_' + str(robot_id) + ': acceleration of every rule and the total')
    figure.tight_layout()
    path = os.path.join(output_folder, 'accelerations_robot_' + str(robot_id) + '.png')
    figure.savefig(path, dpi=120)
    return path


def main():
    parser = argparse.ArgumentParser(description='Plot a flocking run from a ROS 2 bag')
    parser.add_argument('--bag_name', default=None, help='name of a bag in <log_folder>/bags')
    parser.add_argument('--bag_folder', default=None, help='full path of a bag saved somewhere else')
    parser.add_argument('--robot', type=int, default=0, help='robot used for the detailed plots')
    parser.add_argument('--show', action='store_true', help='open the plot windows')
    parser.add_argument('--plot_folder', default=None,
                        help='folder for the pictures (default: <log_folder>/plots/<bag name>)')
    arguments = parser.parse_args()

    if arguments.bag_name is None and arguments.bag_folder is None:
        parser.error('give --bag_name or --bag_folder')

    if not arguments.show:
        matplotlib.use('Agg')   # only save pictures, do not open windows
    from matplotlib import pyplot

    log_folder = read_log_folder()
    if arguments.bag_folder is None:
        bag_folder = os.path.join(log_folder, 'bags', arguments.bag_name)
    else:
        bag_folder = os.path.abspath(os.path.expanduser(arguments.bag_folder)).rstrip('/')
    print('Reading ' + bag_folder)

    robots = read_bag(bag_folder)
    if arguments.robot not in robots:
        print('Robot ' + str(arguments.robot) + ' is not in the bag')
        return

    for robot in robots.values():
        compute_actual_velocity(robot)
    remove_data_after_last_command(robots)
    shift_all_times(robots)

    if arguments.plot_folder is None:
        output_folder = os.path.join(log_folder, 'plots', os.path.basename(bag_folder))
    else:
        output_folder = os.path.abspath(os.path.expanduser(arguments.plot_folder))
    os.makedirs(output_folder, exist_ok=True)

    saved_files = []
    saved_files.append(plot_all_velocities(robots, output_folder, pyplot))
    saved_files.append(plot_one_robot_velocity(robots, arguments.robot, output_folder, pyplot))
    saved_files.append(plot_accelerations(robots, arguments.robot, output_folder, pyplot))

    print('Saved plots:')
    for path in saved_files:
        print('  ' + path)

    if arguments.show:
        pyplot.show()


if __name__ == '__main__':
    main()
