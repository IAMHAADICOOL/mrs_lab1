# This file contains obstacle avoidance. There is no ROS code in this file.
#
# The robots in Stage have no distance sensors, so we "fake" their eyes:
# we look at the map (the same black-and-white bitmap that Stage uses) and
# check if there is a wall at some point in front of the robot.
#
# Two methods from the lecture are implemented, so they can be compared:
#
#   1) "steer" - Steer to avoid (Reynolds' preferred method)
#        - Look ahead along the direction of travel, inside a tube that is
#          as wide as the robot's clearance.
#        - If a wall is inside the tube, find the closest direction that is
#          free (the "silhouette edge" of the obstacle, enlarged by the
#          clearance) and steer sideways towards it.
#        - Plus a "touch sensor": if a wall is very close, push away from it.
#
#   2) "field" - Force field
#        - Every wall point near the robot pushes the robot away, with a
#          strength that depends only on the distance (the bump-shaped curve
#          on the slide). It does not care where the robot is going.

import math

from reynolds_flocking.reynolds_rules import vector_length


class ObstacleMap:
    """The map of the world, made from the ROS /map message (OccupancyGrid).

    The map is a grid of small squares (cells). Each cell is either free
    (value 0) or a wall (value 100). Cell (0, 0) is in the bottom-left corner.
    """

    def __init__(self, occupancy_grid_message):
        info = occupancy_grid_message.info
        self.resolution = info.resolution          # size of one cell in metres
        self.width = info.width                    # number of cells in x
        self.height = info.height                  # number of cells in y
        self.origin_x = info.origin.position.x     # world x of the bottom-left corner
        self.origin_y = info.origin.position.y     # world y of the bottom-left corner
        self.cells = occupancy_grid_message.data   # one long list, row after row

    def is_wall(self, x, y):
        """Returns True if the world point (x, y) is inside a wall."""
        column = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))

        # Everything outside the map counts as a wall
        if column < 0 or column >= self.width:
            return True
        if row < 0 or row >= self.height:
            return True

        index = row * self.width + column
        value = self.cells[index]
        # 100 = wall, 0 = free, -1 = unknown. Treat 50 and above as wall.
        if value >= 50:
            return True
        return False

    def free_distance(self, x, y, direction_angle, look_ahead, clearance):
        """How far can the robot travel in 'direction_angle' before its tube
        touches a wall?

        The tube is the yellow band from the slide: a centre line plus a line
        on the left and a line on the right, 'clearance' metres away from the
        centre. Near the robot the tube is narrower (it grows from 0 up to
        'clearance'), so a wall right next to the robot does not block every
        direction. That case is handled by the touch sensor instead.

        Returns 'look_ahead' if the whole tube is free.
        """
        direction_x = math.cos(direction_angle)
        direction_y = math.sin(direction_angle)
        # Vector pointing to the left of the direction of travel
        # we are taking normal direction here, and we know that 
        # the normal vector is (-y, x) for a vector (x, y)
        side_x = -direction_y
        side_y = direction_x

        k = 1 # relation between distance and the width of the tube, 
        step = self.resolution
        distance = step
        while distance <= look_ahead:
            # The tube starts pointed (like a pencil tip) and then gets wider
            width = k * distance
            if width < clearance:
                tube_half_width = width
            else:
                tube_half_width = clearance

            # Check the right line, the centre line and the left line
            for side_offset in [-tube_half_width, 0.0, tube_half_width]:
                point_x = x + direction_x * distance + side_x * side_offset
                point_y = y + direction_y * distance + side_y * side_offset
                if self.is_wall(point_x, point_y):
                    return distance

            distance = distance + step

        return look_ahead

    def nearest_wall_point(self, x, y, search_radius):
        """Find the wall cell closest to (x, y), looking only inside a square
        of size 'search_radius' around the robot.

        Returns (found, wall_x, wall_y, distance).
        """
        found = False
        best_x = 0.0
        best_y = 0.0
        best_distance = search_radius

        cells_to_search = int(math.ceil(search_radius / self.resolution))
        robot_column = int(math.floor((x - self.origin_x) / self.resolution))
        robot_row = int(math.floor((y - self.origin_y) / self.resolution))

        for row in range(robot_row - cells_to_search, robot_row + cells_to_search + 1):
            for column in range(robot_column - cells_to_search, robot_column + cells_to_search + 1):
                # Centre of this cell in world coordinates
                # Adding half a cell lands in the middle of the square
                cell_x = self.origin_x + (column + 0.5) * self.resolution
                cell_y = self.origin_y + (row + 0.5) * self.resolution
                # this granularity is for computing the distance variable
                
                if not self.is_wall(cell_x, cell_y):
                    continue

                distance = vector_length(cell_x - x, cell_y - y)
                if distance < best_distance:
                    found = True
                    best_distance = distance
                    best_x = cell_x
                    best_y = cell_y

        return found, best_x, best_y, best_distance


def steer_to_avoid(obstacle_map, robot, look_ahead, clearance, weight_obstacle):
    """Method 1: Steer to avoid (picture with circles A, B, C on the slide).
    1. Look ahead along the heading. If the tube is free -> do nothing.
       (Obstacles behind or beside us are ignored, so obstacles we already
       passed have no effect.)
    2. If the tube hits a wall, try directions turned more and more to the
       left and to the right (20, 40, 60 ... degrees). The first direction
       with a free tube is the edge of the obstacle's silhouette.
    3. Accelerate towards that free direction. This mostly pushes the robot
       SIDEWAYS (steering), instead of only slowing it down.
    4. The closer the wall, the stronger the steering ("urgency", squared).
    """
    heading = robot.heading
    distance_ahead = obstacle_map.free_distance(robot.x, robot.y, heading, look_ahead, clearance)

    # Nothing in front of us
    if distance_ahead >= look_ahead:
        return 0.0, 0.0

    # Urgency: 0 when the wall is at the end of the tube, 1 when it touches us.
    # It is squared, so a far wall gives only a very gentle push (no jumping
    # on/off when the wall is just at the tip of the tube) and a close wall
    # gives a strong push.
    closeness = 1.0 - distance_ahead / look_ahead
    urgency = closeness * closeness
    # Without urgency, the push would be on or off.
    # weight_obstacle is fixed. It's the largest push the robot can get, 1.0 m/s².
    # urgency changes every step. It scales that 
    # push by how close the wall is ahead, from 0 to 1.  
    
    # Search for the closest free direction, turning left and right
    best_angle = heading
    best_free_distance = distance_ahead
    found_free_direction = False

    turn_degrees = 20
    while turn_degrees <= 180 and not found_free_direction:
        turn = math.radians(turn_degrees)

        # Try turning left first, then right
        for candidate_angle in [heading + turn, heading - turn]:
            free = obstacle_map.free_distance(robot.x, robot.y, candidate_angle, look_ahead, clearance)
            if free > best_free_distance:
                best_free_distance = free
                best_angle = candidate_angle
            if free >= look_ahead:
                found_free_direction = True
                break

        turn_degrees = turn_degrees + 20

    # Desired direction (unit vector) minus current direction (unit vector)
    # gives the change of direction we want: the steering vector.
    steer_x = math.cos(best_angle) - math.cos(heading)
    steer_y = math.sin(best_angle) - math.sin(heading)
    # when no better angle could be found inside the loop
    # and therefore, best_anlge == heading, then steer_x and steer_y are both 0.0
    steer_length = vector_length(steer_x, steer_y)
    if steer_length < 0.000001:
        return 0.0, 0.0
    # the above check and return prevents division by zero in the next line
    # unit vector in steer direction is (steer_x + steer_y)/steer_length.
    # the following are the components of it 
    acceleration_x = weight_obstacle * urgency * steer_x / steer_length
    acceleration_y = weight_obstacle * urgency * steer_y / steer_length
    return acceleration_x, acceleration_y


def touch_sensor(obstacle_map, robot, touch_distance, weight_obstacle):
    """Emulated touch sensor: if a wall is closer than 'touch_distance',
    push straight away from the nearest wall point. The closer, the harder.
    """
    found, wall_x, wall_y, distance = obstacle_map.nearest_wall_point(robot.x, robot.y, touch_distance)
    if not found:
        return 0.0, 0.0

    # to stop division by zero in the next line
    if distance < 0.001:
        distance = 0.001

    # Unit vector pointing from the wall to the robot (away from the wall)
    away_x = (robot.x - wall_x) / distance
    away_y = (robot.y - wall_y) / distance

    strength = weight_obstacle * (1.0 - distance / touch_distance)
    return strength * away_x, strength * away_y


def force_field(obstacle_map, robot, field_width, weight_obstacle):
    """Method 2: Force field (the bump-shaped curve on the slide).

    The nearest wall point pushes the robot away. The strength follows

        strength(d) = (d / w) * exp( 0.5 * (1 - (d / w)^2) )

    which is 0 at the wall centre, has its maximum (= 1) at d = w and then
    fades out. We only look up to 3 * w away; beyond that it is almost 0.

    Known problems (from the slide), visible in the simulation:
      - It always acts, even for walls we already passed.
      - Driving straight at a wall it only slows the robot down instead of
        steering it to the side.
    """
    search_radius = 3.0 * field_width # how far to look for the walls
    found, wall_x, wall_y, distance = obstacle_map.nearest_wall_point(robot.x, robot.y, search_radius)
    if not found:
        return 0.0, 0.0

    if distance < 0.001:
        distance = 0.001

    away_x = (robot.x - wall_x) / distance
    away_y = (robot.y - wall_y) / distance

    ratio = distance / field_width
    strength = weight_obstacle * ratio * math.exp(0.5 * (1.0 - ratio * ratio))
    return strength * away_x, strength * away_y
