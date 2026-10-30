# Draws graphs from a CSV log written by the flocking node.
#
# Usage:
#     ros2 run reynolds_flocking plot_flocking                 (newest log in ~/flocking_logs)
#     ros2 run reynolds_flocking plot_flocking --csv_file my_run.csv      (a specific log)
#     ros2 run reynolds_flocking plot_flocking --csv_file my_run.csv --robot 3
#     ros2 run reynolds_flocking plot_flocking --show          (also open the windows)
#     ros2 run reynolds_flocking plot_flocking --plot_folder ~/my_plots
#
# It saves these pictures in a folder next to the CSV file, or in
# 'plot_folder' if you give one:
#     velocities_all_robots.png  commanded vs. actual speed, one small graph per robot
#     velocity_robot_X.png       commanded vs. actual vx and vy of one robot
#     accelerations_robot_X.png  separation, alignment, cohesion, obstacle and total
#     swarm_metrics.png          numbers that describe the whole swarm over time
#     trajectories.png           the path of every robot seen from above

import argparse
import csv
import glob
import math
import os

import matplotlib
import yaml


def find_newest_csv(folder):
    folder = os.path.expanduser(folder)
    csv_files = glob.glob(os.path.join(folder, '*.csv'))
    if len(csv_files) == 0:
        return None
    newest_file = csv_files[0]
    for csv_file in csv_files:
        if os.path.getmtime(csv_file) > os.path.getmtime(newest_file):
            newest_file = csv_file
    return newest_file


def read_parameters(csv_path):
    """Read the _params.yaml file saved next to the CSV (if it exists)."""
    parameters_path = csv_path.replace('.csv', '_params.yaml')
    if not os.path.exists(parameters_path):
        return {}
    with open(parameters_path, 'r') as parameters_file:
        return yaml.safe_load(parameters_file)


def read_log(csv_path):
    """Returns:
        robots: dictionary robot_id -> dictionary column_name -> list of numbers
        steps:  list of control steps, each step is a list of rows (one per robot)
    """
    robots = {}
    steps = []
    current_step = []

    with open(csv_path, 'r') as log_file:
        reader = csv.DictReader(log_file)
        for row in reader:
            robot_id = int(row['robot'])

            # Turn every value into a number
            numbers = {}
            for column_name in row:
                numbers[column_name] = float(row[column_name])

            if robot_id not in robots:
                robots[robot_id] = {}
                for column_name in numbers:
                    robots[robot_id][column_name] = []
            for column_name in numbers:
                robots[robot_id][column_name].append(numbers[column_name])

            # Robot 0 is always written first in every control step
            if robot_id == 0 and len(current_step) > 0:
                steps.append(current_step)
                current_step = []
            current_step.append(numbers)

    if len(current_step) > 0:
        steps.append(current_step)
    return robots, steps


def length(x, y):
    return math.sqrt(x * x + y * y)


def compute_swarm_metrics(steps, neighbour_radius):
    """For every control step compute:
        order          how well the robots move in the same direction
                       (1 = all parallel, 0 = random directions)
        min_distance   the smallest distance between any two robots
        nearest        average distance from each robot to its closest robot
        groups         how many separate groups (robots closer than the
                       neighbour radius are in the same group)
        neighbours     average number of neighbours a robot sees
    """
    metrics = {'time': [], 'order': [], 'min_distance': [], 'nearest': [],
               'groups': [], 'neighbours': []}

    for step in steps:
        number_of_robots = len(step)
        if number_of_robots < 2:
            continue

        # --- order (polarisation) ---
        sum_x = 0.0
        sum_y = 0.0
        for row in step:
            speed = length(row['measured_vx'], row['measured_vy'])
            if speed > 0.000001:
                sum_x = sum_x + row['measured_vx'] / speed
                sum_y = sum_y + row['measured_vy'] / speed
        order = length(sum_x, sum_y) / number_of_robots

        # --- distances ---
        smallest_distance = 1000000.0
        sum_of_nearest = 0.0
        for i in range(number_of_robots):
            nearest_for_this_robot = 1000000.0
            for j in range(number_of_robots):
                if i == j:
                    continue
                distance = length(step[i]['x'] - step[j]['x'], step[i]['y'] - step[j]['y'])
                if distance < nearest_for_this_robot:
                    nearest_for_this_robot = distance
            sum_of_nearest = sum_of_nearest + nearest_for_this_robot
            if nearest_for_this_robot < smallest_distance:
                smallest_distance = nearest_for_this_robot

        # --- groups: flood fill over "closer than neighbour_radius" ---
        group_of_robot = []
        for i in range(number_of_robots):
            group_of_robot.append(-1)
        number_of_groups = 0
        for start in range(number_of_robots):
            if group_of_robot[start] != -1:
                continue
            group_of_robot[start] = number_of_groups
            to_visit = [start]
            while len(to_visit) > 0:
                i = to_visit.pop()
                for j in range(number_of_robots):
                    if group_of_robot[j] != -1:
                        continue
                    distance = length(step[i]['x'] - step[j]['x'], step[i]['y'] - step[j]['y'])
                    if distance <= neighbour_radius:
                        group_of_robot[j] = number_of_groups
                        to_visit.append(j)
            number_of_groups = number_of_groups + 1

        # --- neighbours ---
        sum_of_neighbours = 0.0
        for row in step:
            sum_of_neighbours = sum_of_neighbours + row['neighbours']

        metrics['time'].append(step[0]['time'])
        metrics['order'].append(order)
        metrics['min_distance'].append(smallest_distance)
        metrics['nearest'].append(sum_of_nearest / number_of_robots)
        metrics['groups'].append(number_of_groups)
        metrics['neighbours'].append(sum_of_neighbours / number_of_robots)

    return metrics


def average_of_second_half(values):
    """Average over the second half of the run (after the swarm has settled)."""
    half = len(values) // 2
    second_half = values[half:]
    if len(second_half) == 0:
        return 0.0
    total = 0.0
    for value in second_half:
        total = total + value
    return total / len(second_half)


def plot_all_velocities(robots, output_folder, pyplot):
    robot_ids = sorted(robots.keys())
    number_of_robots = len(robot_ids)
    columns = 2
    rows = int(math.ceil(number_of_robots / columns))

    figure, axes = pyplot.subplots(rows, columns, figsize=(12, 2.2 * rows), sharex=True, squeeze=False)
    for index in range(rows * columns):
        axis = axes[index // columns][index % columns]
        if index >= number_of_robots:
            axis.axis('off')
            continue
        robot = robots[robot_ids[index]]

        commanded_speed = []
        measured_speed = []
        for k in range(len(robot['time'])):
            commanded_speed.append(length(robot['command_vx'][k], robot['command_vy'][k]))
            measured_speed.append(length(robot['measured_vx'][k], robot['measured_vy'][k]))

        axis.plot(robot['time'], commanded_speed, label='commanded')
        axis.plot(robot['time'], measured_speed, '--', label='actual (odom)')
        axis.set_title('robot_' + str(robot_ids[index]), fontsize=9)
        axis.set_ylabel('speed [m/s]')
        axis.grid(True)
    axes[0][0].legend(fontsize=8)
    axes[rows - 1][0].set_xlabel('time [s]')
    axes[rows - 1][1].set_xlabel('time [s]')
    figure.suptitle('Commanded vs. actual speed of every robot')
    figure.tight_layout()
    path = os.path.join(output_folder, 'velocities_all_robots.png')
    figure.savefig(path, dpi=120)
    return path


def plot_one_robot_velocity(robots, robot_id, output_folder, pyplot):
    robot = robots[robot_id]
    figure, axes = pyplot.subplots(2, 1, figsize=(10, 6), sharex=True)

    axes[0].plot(robot['time'], robot['command_vx'], label='commanded vx')
    axes[0].plot(robot['time'], robot['measured_vx'], '--', label='actual vx (odom)')
    axes[0].set_ylabel('vx [m/s]')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(robot['time'], robot['command_vy'], label='commanded vy')
    axes[1].plot(robot['time'], robot['measured_vy'], '--', label='actual vy (odom)')
    axes[1].set_ylabel('vy [m/s]')
    axes[1].set_xlabel('time [s]')
    axes[1].legend()
    axes[1].grid(True)

    figure.suptitle('robot_' + str(robot_id) + ': commanded vs. actual velocity')
    figure.tight_layout()
    path = os.path.join(output_folder, 'velocity_robot_' + str(robot_id) + '.png')
    figure.savefig(path, dpi=120)
    return path


def plot_accelerations(robots, robot_id, output_folder, pyplot):
    robot = robots[robot_id]
    names = ['separation', 'alignment', 'cohesion', 'obstacle', 'total']

    figure, axes = pyplot.subplots(3, 1, figsize=(10, 9), sharex=True)
    for name in names:
        x_values = robot[name + '_x']
        y_values = robot[name + '_y']
        sizes = []
        for k in range(len(x_values)):
            sizes.append(length(x_values[k], y_values[k]))

        if name == 'total':
            line_width = 2.0
            line_color = 'black'
        else:
            line_width = 1.0
            line_color = None

        axes[0].plot(robot['time'], x_values, label=name, linewidth=line_width, color=line_color)
        axes[1].plot(robot['time'], y_values, label=name, linewidth=line_width, color=line_color)
        axes[2].plot(robot['time'], sizes, label=name, linewidth=line_width, color=line_color)

    axes[0].set_ylabel('a_x [m/s²]')
    axes[1].set_ylabel('a_y [m/s²]')
    axes[2].set_ylabel('|a| [m/s²]')
    axes[2].set_xlabel('time [s]')
    for axis in axes:
        axis.grid(True)
    axes[0].legend(loc='upper right', fontsize=8)
    figure.suptitle('robot_' + str(robot_id) + ': acceleration of every rule and the total')
    figure.tight_layout()
    path = os.path.join(output_folder, 'accelerations_robot_' + str(robot_id) + '.png')
    figure.savefig(path, dpi=120)
    return path


def plot_metrics(metrics, output_folder, pyplot):
    figure, axes = pyplot.subplots(4, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(metrics['time'], metrics['order'])
    axes[0].set_ylabel('order (0..1)')
    axes[0].set_ylim(0.0, 1.05)

    axes[1].plot(metrics['time'], metrics['min_distance'], label='smallest distance')
    axes[1].plot(metrics['time'], metrics['nearest'], label='average nearest-neighbour distance')
    axes[1].set_ylabel('distance [m]')
    axes[1].legend(fontsize=8)

    axes[2].plot(metrics['time'], metrics['groups'])
    axes[2].set_ylabel('number of groups')

    axes[3].plot(metrics['time'], metrics['neighbours'])
    axes[3].set_ylabel('avg. neighbours seen')
    axes[3].set_xlabel('time [s]')

    for axis in axes:
        axis.grid(True)
    figure.suptitle('Swarm metrics')
    figure.tight_layout()
    path = os.path.join(output_folder, 'swarm_metrics.png')
    figure.savefig(path, dpi=120)
    return path


def plot_trajectories(robots, output_folder, pyplot):
    figure, axis = pyplot.subplots(figsize=(7, 7))
    for robot_id in sorted(robots.keys()):
        robot = robots[robot_id]
        line = axis.plot(robot['x'], robot['y'], linewidth=0.8, label='robot_' + str(robot_id))
        color = line[0].get_color()
        # circle = start, triangle = end
        axis.plot(robot['x'][0], robot['y'][0], 'o', color=color)
        axis.plot(robot['x'][-1], robot['y'][-1], '^', color=color)
    axis.set_aspect('equal')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.grid(True)
    axis.legend(fontsize=7, loc='upper right')
    axis.set_title('Trajectories (o = start, ^ = end)')
    figure.tight_layout()
    path = os.path.join(output_folder, 'trajectories.png')
    figure.savefig(path, dpi=120)
    return path


def main():
    parser = argparse.ArgumentParser(description='Plot a flocking CSV log')
    parser.add_argument('--csv_file', default=None,
                        help='CSV log file (default: newest one in ~/flocking_logs)')
    parser.add_argument('--robot', type=int, default=0, help='robot used for the detailed plots')
    parser.add_argument('--show', action='store_true', help='open the plot windows')
    parser.add_argument('--plot_folder', default=None,
                        help='folder for the pictures (default: a folder next to the CSV file)')
    arguments = parser.parse_args()

    if not arguments.show:
        matplotlib.use('Agg')   # only save pictures, do not open windows
    from matplotlib import pyplot

    csv_path = arguments.csv_file
    if csv_path is None:
        csv_path = find_newest_csv('~/flocking_logs')
        if csv_path is None:
            print('No CSV file found in ~/flocking_logs')
            return
    csv_path = os.path.abspath(os.path.expanduser(csv_path))
    print('Reading ' + csv_path)

    parameters = read_parameters(csv_path)
    neighbour_radius = 1.0
    if 'neighbour_radius' in parameters:
        neighbour_radius = float(parameters['neighbour_radius'])

    robots, steps = read_log(csv_path)
    if arguments.robot not in robots:
        print('Robot ' + str(arguments.robot) + ' is not in the log')
        return

    if arguments.plot_folder is None:
        output_folder = csv_path.replace('.csv', '_plots')
    else:
        output_folder = os.path.abspath(os.path.expanduser(arguments.plot_folder))
    os.makedirs(output_folder, exist_ok=True)

    metrics = compute_swarm_metrics(steps, neighbour_radius)

    saved_files = []
    saved_files.append(plot_all_velocities(robots, output_folder, pyplot))
    saved_files.append(plot_one_robot_velocity(robots, arguments.robot, output_folder, pyplot))
    saved_files.append(plot_accelerations(robots, arguments.robot, output_folder, pyplot))
    saved_files.append(plot_metrics(metrics, output_folder, pyplot))
    saved_files.append(plot_trajectories(robots, output_folder, pyplot))

    print('')
    print('Summary (average over the second half of the run):')
    print('  order (1 = all aligned)       : ' + str(round(average_of_second_half(metrics['order']), 3)))
    print('  avg nearest-neighbour distance: ' + str(round(average_of_second_half(metrics['nearest']), 3)) + ' m')
    print('  number of groups              : ' + str(round(average_of_second_half(metrics['groups']), 2)))
    print('  avg neighbours seen           : ' + str(round(average_of_second_half(metrics['neighbours']), 2)))
    print('  smallest distance (whole run) : ' + str(round(min(metrics['min_distance']), 3)) + ' m')
    print('')
    print('Saved plots:')
    for path in saved_files:
        print('  ' + path)

    if arguments.show:
        pyplot.show()


if __name__ == '__main__':
    main()
