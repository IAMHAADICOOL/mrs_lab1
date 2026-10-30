# The formula from the lecture (slide "Combining accelerations") is:
#
#   acceleration_i =   c_a / |N_i| * sum( v_j ) - v_i                         (alignment)
#                    + c_c / |N_i| * sum( x_j ) - x_i                         (cohesion)
#                    - c_s / |N_i| * sum( (x_j - x_i) / |x_j - x_i|^2 )       (separation)
#
# where N_i is the list of neighbours of robot i and |N_i| is how many there are.
#
# Every vector is kept as two separate numbers: an x part and a y part.

import math


class RobotState:
    """Everything we know about one robot."""

    def __init__(self, robot_id):
        self.robot_id = robot_id

        # Position in the world (metres), read from /robot_X/odom
        self.x = 0.0
        self.y = 0.0

        # Orientation of the robot body in the world (radians), read from odom.
        # We never rotate the robots, so this stays around 0.
        self.yaw = 0.0

        # Measured velocity in the world frame (metres per second), read from odom
        self.measured_vx = 0.0
        self.measured_vy = 0.0

        # The velocity WE command to this robot. We change it a little bit
        # every step using the acceleration (v = v + a * dt).
        self.command_vx = 0.0
        self.command_vy = 0.0

        # Direction of travel (radians). Used for the field of view.
        self.heading = 0.0

        # True once we have received at least one odom message
        self.has_odom = False


def wrap_angle(angle):
    """Bring any angle into the range -pi ... +pi."""
    while angle > math.pi:
        angle = angle - 2.0 * math.pi
    while angle < -math.pi:
        angle = angle + 2.0 * math.pi
    return angle


def vector_length(x, y):
    """Length of the vector (x, y)."""
    return math.sqrt(x * x + y * y)


def limit_vector(x, y, max_length):
    """If the vector is longer than max_length, shrink it to max_length.
    The direction stays the same."""
    length = vector_length(x, y)
    if length > max_length and length > 0.0:
        scale = max_length / length
        x = x * scale
        y = y * scale
    return x, y


def is_neighbour(my_robot, other_robot, neighbour_radius, fov_degrees):
    """Returns True if other_robot is inside the neighbourhood of my_robot.

    The neighbourhood is a circle of radius 'neighbour_radius' around the
    robot, with a blind spot behind it. The robot can only see inside
    the field of view (FOV) angle, measured from its heading.
    """
    dx = other_robot.x - my_robot.x
    dy = other_robot.y - my_robot.y
    distance = vector_length(dx, dy)

    # Rule 1: too far away -> not a neighbour
    if distance > neighbour_radius:
        return False

    # If the full circle is visible, every robot that is close enough counts
    if fov_degrees >= 360.0:
        return True

    # Rule 2: must be inside the field of view.
    # angle_to_other is the direction from me to the other robot, measured along the world x-axis
    angle_to_other = math.atan2(dy, dx)
    
    # How far is that direction from where I am heading?
    angle_difference = wrap_angle(angle_to_other - my_robot.heading)
    # angle_difference subtracts my heading from that, giving "how far is the other robot from straight ahead?"
    # positive means it's to my left
    # negative means it's to my right
    half_fov = math.radians(fov_degrees) / 2.0

    # if the other robot is within the fov, it can either be on right or on left, 
    # so we take the absolute value of the angle difference and compare it to half the fov
    if abs(angle_difference) <= half_fov:
        return True
    return False


def find_neighbours(my_robot, all_robots, neighbour_radius, fov_degrees):
    """Returns the list N_i: all robots that my_robot can see."""
    neighbours = []
    for other_robot in all_robots:
        # A robot is not its own neighbour (j != i in the formula)
        if other_robot.robot_id == my_robot.robot_id:
            continue
        if is_neighbour(my_robot, other_robot, neighbour_radius, fov_degrees):
            neighbours.append(other_robot)
    return neighbours


def separation_rule(my_robot, neighbours, weight_separation, distance_power):
    """Separation: push away from neighbours.

    Formula:  a_sep = - c_s / |N_i| * sum( (x_j - x_i) / |x_j - x_i|^p )

    With p = 2 (the lecture formula) the push from one neighbour has size
    1 / distance, so close neighbours push much harder than far ones.
    """
    number_of_neighbours = len(neighbours)
    if number_of_neighbours == 0:
        return 0.0, 0.0

    sum_x = 0.0
    sum_y = 0.0
    for other_robot in neighbours:
        # (x_j - x_i): vector pointing from me to the neighbour
        dx = other_robot.x - my_robot.x
        dy = other_robot.y - my_robot.y
        distance = vector_length(dx, dy)

        # Never divide by zero if two robots are exactly on top of each other
        if distance < 0.01:
            distance = 0.01

        # (x_j - x_i) / |x_j - x_i|^p
        divider = distance ** distance_power
        sum_x = sum_x + dx / divider
        sum_y = sum_y + dy / divider

    # The minus sign turns "towards the neighbour" into "away from the neighbour"
    acceleration_x = -weight_separation / number_of_neighbours * sum_x
    acceleration_y = -weight_separation / number_of_neighbours * sum_y
    return acceleration_x, acceleration_y


def alignment_rule(my_robot, neighbours, weight_alignment):
    """Alignment: match the average velocity of the neighbours.

    Formula:  a_ali = c_a * ( (1 / |N_i|) * sum(v_j)  -  v_i )
                                 average velocity       my velocity
    """
    number_of_neighbours = len(neighbours)
    if number_of_neighbours == 0:
        return 0.0, 0.0

    sum_vx = 0.0
    sum_vy = 0.0
    for other_robot in neighbours:
        sum_vx = sum_vx + other_robot.measured_vx
        sum_vy = sum_vy + other_robot.measured_vy

    average_vx = sum_vx / number_of_neighbours
    average_vy = sum_vy / number_of_neighbours

    # Difference between what the neighbours do and what I do
    acceleration_x = weight_alignment * (average_vx - my_robot.command_vx)
    acceleration_y = weight_alignment * (average_vy - my_robot.command_vy)
    return acceleration_x, acceleration_y


def cohesion_rule(my_robot, neighbours, weight_cohesion):
    """Cohesion: move towards the centre of the neighbours.

    Formula:  a_coh = c_c * ( (1 / |N_i|) * sum(x_j)  -  x_i )
                                local centre of mass     my position
    """
    number_of_neighbours = len(neighbours)
    if number_of_neighbours == 0:
        return 0.0, 0.0

    sum_x = 0.0
    sum_y = 0.0
    for other_robot in neighbours:
        sum_x = sum_x + other_robot.x
        sum_y = sum_y + other_robot.y

    centre_x = sum_x / number_of_neighbours
    centre_y = sum_y / number_of_neighbours

    # Vector from me to the local centre of mass
    acceleration_x = weight_cohesion * (centre_x - my_robot.x)
    acceleration_y = weight_cohesion * (centre_y - my_robot.y)
    return acceleration_x, acceleration_y
