# The ROS 2 node that makes the robots flock.
#
# One node controls all robots, but every robot only uses information about
# the robots inside its own neighbourhood (distance + field of view). So the
# logic is the same as if every robot had its own small computer.
#
# Every control step (20 times per second) for every robot:
#   1. find its neighbours
#   2. compute separation, alignment, cohesion and obstacle accelerations
#   3. combine them into one acceleration
#   4. update the velocity a little bit:  v = v + a * dt
#   5. send the velocity to the robot on /robot_X/cmd_vel

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry

from reynolds_flocking.reynolds_rules import RobotState
from reynolds_flocking.reynolds_rules import find_neighbours
from reynolds_flocking.reynolds_rules import separation_rule
from reynolds_flocking.reynolds_rules import alignment_rule
from reynolds_flocking.reynolds_rules import cohesion_rule
from reynolds_flocking.reynolds_rules import vector_length
from reynolds_flocking.reynolds_rules import limit_vector
from reynolds_flocking.obstacle_avoidance import ObstacleMap
from reynolds_flocking.obstacle_avoidance import steer_to_avoid
from reynolds_flocking.obstacle_avoidance import touch_sensor
from reynolds_flocking.obstacle_avoidance import force_field


class FlockingNode(Node):

    def __init__(self):
        super().__init__('reynolds_flocking')

        
        # 1. Read all parameters (values come from config/flocking_params.yaml)
        self.num_robots = self.declare_parameter('num_robots', 10).value
        self.control_rate = self.declare_parameter('control_rate', 20.0).value

        self.neighbour_radius = self.declare_parameter('neighbour_radius', 1.0).value
        self.fov_degrees = self.declare_parameter('fov_degrees', 360.0).value

        self.weight_separation = self.declare_parameter('weight_separation', 0.06).value
        self.weight_alignment = self.declare_parameter('weight_alignment', 1.0).value
        self.weight_cohesion = self.declare_parameter('weight_cohesion', 0.5).value
        self.separation_distance_power = self.declare_parameter('separation_distance_power', 3.0).value

        self.max_speed = self.declare_parameter('max_speed', 0.3).value
        self.min_speed = self.declare_parameter('min_speed', 0.15).value
        self.max_acceleration = self.declare_parameter('max_acceleration', 0.5).value
        self.initial_speed = self.declare_parameter('initial_speed', 0.15).value
        self.random_seed = self.declare_parameter('random_seed', 42).value

        self.obstacle_mode = self.declare_parameter('obstacle_mode', 'steer').value
        self.weight_obstacle = self.declare_parameter('weight_obstacle', 1.0).value
        self.look_ahead = self.declare_parameter('look_ahead', 0.8).value
        self.clearance = self.declare_parameter('clearance', 0.15).value
        self.touch_distance = self.declare_parameter('touch_distance', 0.2).value
        self.field_width = self.declare_parameter('field_width', 0.25).value

        self.combine_mode = self.declare_parameter('combine_mode', 'weighted_sum').value

        # Time between two control steps, in seconds (dt in v = v + a * dt)
        self.dt = 1.0 / self.control_rate

        
        # 2. Create one RobotState for every robot
        self.robots = []
        for robot_id in range(self.num_robots):
            self.robots.append(RobotState(robot_id))

        # The map is received later on the /map topic
        self.obstacle_map = None

        # The robots start standing still, so they have no direction yet.
        # We give each one a small random velocity once all odom has arrived.
        self.started = False
        self.random_generator = random.Random(self.random_seed)

        # 3. Publishers and subscribers for every robot
        self.velocity_publishers = []
        self.separation_publishers = []
        self.alignment_publishers = []
        self.cohesion_publishers = []
        self.obstacle_publishers = []
        self.total_publishers = []

        for robot_id in range(self.num_robots):
            name = 'robot_' + str(robot_id)

            self.velocity_publishers.append(
                self.create_publisher(Twist, '/' + name + '/cmd_vel', 10))

            # Extra topics only for plotting (record them with ros2 bag)
            self.separation_publishers.append(
                self.create_publisher(Vector3Stamped, '/' + name + '/flocking/separation', 10))
            self.alignment_publishers.append(
                self.create_publisher(Vector3Stamped, '/' + name + '/flocking/alignment', 10))
            self.cohesion_publishers.append(
                self.create_publisher(Vector3Stamped, '/' + name + '/flocking/cohesion', 10))
            self.obstacle_publishers.append(
                self.create_publisher(Vector3Stamped, '/' + name + '/flocking/obstacle', 10))
            self.total_publishers.append(
                self.create_publisher(Vector3Stamped, '/' + name + '/flocking/total_acceleration', 10))

            # All robots use the same callback. The callback finds out which
            # robot the message belongs to from msg.header.frame_id.
            self.create_subscription(Odometry, '/' + name + '/odom', self.odom_callback, 10)

        # The map server sends the map only once, and keeps it for late
        # subscribers ("transient local"). Our subscriber must ask for that too.
        # the following QoS settings are required after checking the 
        # topic settings in the terminal.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # Timer: call control_step() 'control_rate' times per second
        self.create_timer(self.dt, self.control_step)

        self.get_logger().info(
            'Flocking started for ' + str(self.num_robots) + ' robots, '
            + 'radius=' + str(self.neighbour_radius) + ' m, '
            + 'FOV=' + str(self.fov_degrees) + ' deg, '
            + 'obstacle_mode=' + self.obstacle_mode + ', '
            + 'combine_mode=' + self.combine_mode)

    # Callbacks: store incoming data
    def odom_callback(self, msg):
        """Save the position and velocity of one robot.
        frame_id looks like 'robot_3/odom' -> this message is for robot 3."""
        robot_name = msg.header.frame_id.split('/')[0]
        robot_id = int(robot_name.replace('robot_', ''))
        if robot_id >= self.num_robots:
            return
        robot = self.robots[robot_id]

        robot.x = msg.pose.pose.position.x
        robot.y = msg.pose.pose.position.y

        # Turn the quaternion into a yaw angle (rotation around z)
        q = msg.pose.pose.orientation
        robot.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Stage reports velocity in the robot's own frame. Rotate it into the
        # world frame. (yaw is 0 for us, so this changes nothing, but it is correct.)
        body_vx = msg.twist.twist.linear.x
        body_vy = msg.twist.twist.linear.y
        robot.measured_vx = math.cos(robot.yaw) * body_vx - math.sin(robot.yaw) * body_vy
        robot.measured_vy = math.sin(robot.yaw) * body_vx + math.cos(robot.yaw) * body_vy

        robot.has_odom = True

    def map_callback(self, msg):
        self.obstacle_map = ObstacleMap(msg)
        self.get_logger().info(
            'Map received: ' + str(msg.info.width) + ' x ' + str(msg.info.height)
            + ' cells, ' + str(msg.info.resolution) + ' m per cell')

    # The main loop
    def control_step(self):
        # Wait until we know where every robot is
        for robot in self.robots:
            if not robot.has_odom:
                self.get_logger().info('Waiting for odom from all robots', throttle_duration_sec=2.0)
                return

        if not self.started:
            self.give_random_start_velocities()
            self.started = True

        if self.obstacle_mode != 'none' and self.obstacle_map is None:
            self.get_logger().warn('No map received yet, obstacle avoidance is off for now',
                                   throttle_duration_sec=2.0)

        # Compute the new velocity of every robot first 
        results = []
        for robot in self.robots:
            result = self.compute_robot(robot)
            results.append(result)

        # and only then send them, so every robot used the same snapshot
        for robot_id in range(self.num_robots):
            if not rclpy.ok():
                return
            robot = self.robots[robot_id]
            result = results[robot_id]
            robot.command_vx = result['new_vx']
            robot.command_vy = result['new_vy']
            self.update_heading(robot)
            self.send_velocity(robot)
            self.publish_accelerations(robot_id, result)

    def compute_robot(self, robot):
        """All the flocking math for one robot. Returns a dictionary with
        every acceleration and the new velocity."""

        # 1: who can I see?
        neighbours = find_neighbours(robot, self.robots, self.neighbour_radius, self.fov_degrees)

        # 2: the three Reynolds rules
        separation_x, separation_y = separation_rule(
            robot, neighbours, self.weight_separation, self.separation_distance_power)
        alignment_x, alignment_y = alignment_rule(robot, neighbours, self.weight_alignment)
        cohesion_x, cohesion_y = cohesion_rule(robot, neighbours, self.weight_cohesion)

        # 3: obstacle avoidance
        obstacle_x, obstacle_y = self.compute_obstacle_acceleration(robot)

        # 4: combine everything into one acceleration
        if self.combine_mode == 'priority':
            total_x, total_y = self.combine_with_priority(
                [[obstacle_x, obstacle_y],
                 [separation_x, separation_y],
                 [alignment_x, alignment_y],
                 [cohesion_x, cohesion_y]])
        else:
            # Weighted sum (the weights are already inside each rule)
            total_x = separation_x + alignment_x + cohesion_x + obstacle_x
            total_y = separation_y + alignment_y + cohesion_y + obstacle_y
            # A real robot cannot accelerate faster than its motors allow
            total_x, total_y = limit_vector(total_x, total_y, self.max_acceleration)

        # 5: change the velocity a little bit:  v_new = v_old + a * dt
        new_vx = robot.command_vx + total_x * self.dt
        new_vy = robot.command_vy + total_y * self.dt
        new_vx, new_vy = self.limit_speed(new_vx, new_vy, robot)

        result = {
            'neighbours': len(neighbours),
            'separation_x': separation_x, 'separation_y': separation_y,
            'alignment_x': alignment_x, 'alignment_y': alignment_y,
            'cohesion_x': cohesion_x, 'cohesion_y': cohesion_y,
            'obstacle_x': obstacle_x, 'obstacle_y': obstacle_y,
            'total_x': total_x, 'total_y': total_y,
            'new_vx': new_vx, 'new_vy': new_vy,
        }
        return result

    def compute_obstacle_acceleration(self, robot):
        if self.obstacle_map is None:
            return 0.0, 0.0

        if self.obstacle_mode == 'steer':
            steer_x, steer_y = steer_to_avoid(
                self.obstacle_map, robot, self.look_ahead, self.clearance, self.weight_obstacle)
            touch_x, touch_y = touch_sensor(
                self.obstacle_map, robot, self.touch_distance, self.weight_obstacle)
            return steer_x + touch_x, steer_y + touch_y
        # both are combined so that one covers the blind spots of the other. 
        # The touch sensor is very short range, but it can detect walls that are too close to the robot's side. 
        # The tube is a long range sensor, but it has blind spots on the sides of the robot.

        if self.obstacle_mode == 'field':
            return force_field(self.obstacle_map, robot, self.field_width, self.weight_obstacle)

        # obstacle_mode == 'none'
        return 0.0, 0.0

    def combine_with_priority(self, accelerations_in_priority_order):
        """Prioritised acceleration allocation (Reynolds 1987).

        We have a budget of 'max_acceleration'. The most important request
        (obstacle) is served first, then separation, alignment, cohesion.
        When the budget is used up, the remaining requests are cut off.
        """
        total_x = 0.0
        total_y = 0.0
        budget_left = self.max_acceleration

        for acceleration in accelerations_in_priority_order:
            acceleration_x = acceleration[0]
            acceleration_y = acceleration[1]
            size = vector_length(acceleration_x, acceleration_y)

            if size <= budget_left:
                # Enough budget: take the full request
                total_x = total_x + acceleration_x
                total_y = total_y + acceleration_y
                budget_left = budget_left - size
            else:
                # Not enough: take only the part that still fits, then stop
                if size > 0.0:
                    fraction = budget_left / size
                    total_x = total_x + acceleration_x * fraction
                    total_y = total_y + acceleration_y * fraction
                break

        return total_x, total_y

    def limit_speed(self, vx, vy, robot):
        """Keep the speed between min_speed and max_speed."""
        speed = vector_length(vx, vy)

        if speed > self.max_speed:
            vx = vx / speed * self.max_speed
            vy = vy / speed * self.max_speed
        elif speed < self.min_speed:
            if speed > 0.000001:
                # Same direction, but at least min_speed
                vx = vx / speed * self.min_speed
                vy = vy / speed * self.min_speed
            else:
                # Velocity is exactly zero: keep going along the last heading
                vx = math.cos(robot.heading) * self.min_speed
                vy = math.sin(robot.heading) * self.min_speed

        return vx, vy

    def update_heading(self, robot):
        """The robots are omnidirectional, so the heading is the direction of
        the velocity vector. If the robot is (almost) standing still the
        direction is not defined, so we keep the old heading."""
        speed = vector_length(robot.command_vx, robot.command_vy)
        if speed > 0.01:
            robot.heading = math.atan2(robot.command_vy, robot.command_vx)

    def give_random_start_velocities(self):
        """Every robot starts with a small velocity in a random direction.
        The random generator uses a fixed seed, so every run is the same."""
        for robot in self.robots:
            angle = self.random_generator.uniform(-math.pi, math.pi)
            robot.command_vx = self.initial_speed * math.cos(angle)
            robot.command_vy = self.initial_speed * math.sin(angle)
            robot.heading = angle
        self.get_logger().info('All robots found. Giving random start velocities (seed '
                               + str(self.random_seed) + ').')

    # Output: velocity commands and plotting topics
    def send_velocity(self, robot):
        """Stage expects the velocity in the robot's own frame, so rotate the
        world velocity by -yaw. We never send a rotation (angular.z = 0)."""
        world_vx = robot.command_vx
        world_vy = robot.command_vy

        message = Twist()
        message.linear.x = math.cos(robot.yaw) * world_vx + math.sin(robot.yaw) * world_vy
        message.linear.y = -math.sin(robot.yaw) * world_vx + math.cos(robot.yaw) * world_vy
        message.angular.z = 0.0
        self.velocity_publishers[robot.robot_id].publish(message)

    def publish_accelerations(self, robot_id, result):
        now = self.get_clock().now().to_msg()
        self.publish_vector(self.separation_publishers[robot_id], now,
                            result['separation_x'], result['separation_y'])
        self.publish_vector(self.alignment_publishers[robot_id], now,
                            result['alignment_x'], result['alignment_y'])
        self.publish_vector(self.cohesion_publishers[robot_id], now,
                            result['cohesion_x'], result['cohesion_y'])
        self.publish_vector(self.obstacle_publishers[robot_id], now,
                            result['obstacle_x'], result['obstacle_y'])
        self.publish_vector(self.total_publishers[robot_id], now,
                            result['total_x'], result['total_y'])

    def publish_vector(self, publisher, stamp, x, y):
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = 'map'
        message.vector.x = x
        message.vector.y = y
        message.vector.z = 0.0
        publisher.publish(message)

    def stop_all_robots(self):
        for robot in self.robots:
            self.velocity_publishers[robot.robot_id].publish(Twist())


def main():
    rclpy.init()
    node = FlockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    # Stop the robots before quitting (if ROS is still running)
    if rclpy.ok():
        node.stop_all_robots()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
