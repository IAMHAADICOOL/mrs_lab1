# Reynolds Flocking – How the Code Works

This document explains the `reynolds_flocking` package from start to end in simple words:
how the program starts, what happens every control step, and how every line that
implements a formula maps to the symbols in that formula.

---

## 1. What the package does

Ten (or any number of) robots drive around in the Stage simulator. Every robot:

1. looks at the robots near it (its **neighbourhood**),
2. computes four "wishes" as accelerations:
   - **separation** – "don't bump into my neighbours",
   - **alignment** – "go the same way as my neighbours",
   - **cohesion** – "stay close to my neighbours",
   - **obstacle avoidance** – "don't drive into walls",
3. adds them together into one acceleration,
4. changes its velocity a little bit with that acceleration (no sudden jumps, like a real robot),
5. sends the new velocity to the simulator.

Nobody tells the robots where to go. The flock (all robots moving together in one
direction) **emerges** from these simple local rules.

---

## 2. The files

```
reynolds_flocking/
├── config/flocking_params.yaml        all numbers you can tune (weights, radius, FOV, ...)
├── launch/flocking.launch.py          starts the node with the parameters
├── launch/record_bag.launch.py        records a bag to the log_folder set in the parameters
├── reynolds_flocking/
│   ├── flocking_node.py               ROS part: subscribers, publishers, the main loop
│   ├── reynolds_rules.py              math: neighbours, separation, alignment, cohesion
│   ├── obstacle_avoidance.py          math: reading the map, steer-to-avoid, force field
│   └── plot_flocking.py               draws the graphs from a ROS 2 bag
├── package.xml, setup.py, setup.cfg   ROS package boilerplate
└── FLOCKING_EXPLAINED.md              this file
```

The math files (`reynolds_rules.py`, `obstacle_avoidance.py`) contain **no ROS code**.
They only take numbers in and give numbers out. That keeps the formulas easy to read.

---

## 3. How to run it

### 3.1 Build (only after changing files that are not Python, or adding new files)

```bash
cd ~/University_of_Zagreb/mrs_ws
colcon build --symlink-install
source install/setup.bash
```

Because of `--symlink-install`, changes to the `.py` and `.yaml` files work
immediately, without building again.

### 3.2 Choose the map and the number of robots

Edit `src/sphero_simulation/sphero_stage/launch/launch_params.yaml`:

```yaml
map_name: 'empty_10x10'    # or 'empty_3x3'
num_of_robots: 10
distribution: 'circle'
distribution_list:
  circle:
    center_x: 0
    center_y: 0
    radius: 1              # must be < 1.5 for the 3x3 map, otherwise robots spawn outside the walls
```

(This file was changed from the original `test_10x10`, 6 robots, radius 2.)

### 3.3 Start

Terminal 1 – the simulator (a Stage window opens):

```bash
ros2 launch sphero_stage start.launch.py
```

Terminal 2 – the flocking controller:

```bash
ros2 launch reynolds_flocking flocking.launch.py
```

Stop with `Ctrl+C`. The robots stop by themselves after 5 seconds (the simulator has a
safety "watchdog" that stops robots that receive no commands).

### 3.4 Change parameters for an experiment

Option A – edit `config/flocking_params.yaml` and start the node again.

Option B – change single values from the command line (the YAML stays as it is):

```bash
ros2 run reynolds_flocking flocking_node --ros-args \
    --params-file src/reynolds_flocking/config/flocking_params.yaml \
    -p num_robots:=10 -p fov_degrees:=270.0
```

> **Write decimal numbers with a dot** (`270.0`, not `270`). ROS checks the type:
> a parameter that was declared as a decimal number refuses a whole number.

### 3.5 Record a bag and make the plots

Start the recording in its own terminal, **before** the flocking node:

```bash
ros2 launch reynolds_flocking record_bag.launch.py bag_name:=fov_270
```

This runs `ros2 bag record` for the `cmd_vel`, `odom` and `flocking/` topics of every robot. The
bag is saved in `<log_folder>/bags/fov_270`, a folder that must not exist yet. `log_folder` is set
in `config/flocking_params.yaml` under `logging`. When the run is over, stop the flocking node,
then stop the recording with `Ctrl+C`, because the bag is only finished when the recorder closes
it. Then plot it:

```bash
ros2 run reynolds_flocking plot_flocking --bag_name fov_270
ros2 run reynolds_flocking plot_flocking --bag_name fov_270 --robot 3
ros2 run reynolds_flocking plot_flocking --bag_name fov_270 --show   # also open the windows
ros2 run reynolds_flocking plot_flocking --bag_name fov_270 --plot_folder ~/my_plots
ros2 run reynolds_flocking plot_flocking --bag_folder ~/somewhere/fov_270   # a bag saved elsewhere
```

The pictures are saved in `<log_folder>/plots/fov_270`, or in `--plot_folder` if you give one.

You can also watch the values live in PlotJuggler or `rqt_plot`. The node publishes, for every robot:

| Topic | Meaning |
|---|---|
| `/robot_X/cmd_vel` | commanded velocity |
| `/robot_X/odom` | actual velocity and position (from the simulator) |
| `/robot_X/flocking/separation` | separation acceleration |
| `/robot_X/flocking/alignment` | alignment acceleration |
| `/robot_X/flocking/cohesion` | cohesion acceleration |
| `/robot_X/flocking/obstacle` | obstacle avoidance acceleration |
| `/robot_X/flocking/total_acceleration` | the combined acceleration |

---

## 4. The big picture: what runs when

```
ros2 launch reynolds_flocking flocking.launch.py
 │
 ├─ flocking.launch.py
 │    reads num_of_robots from sphero_stage/launch/launch_params.yaml
 │    starts flocking_node with flocking_params.yaml + num_robots
 │
 └─ flocking_node.py : main()
      rclpy.init()                         start ROS
      node = FlockingNode()                run __init__ once (section 5)
      rclpy.spin(node)                     wait for events, forever:
         │
         ├─ a /robot_X/odom message arrives  ──► odom_callback()   (100 times per second per robot)
         │                                        saves position + velocity of robot X
         │
         ├─ the /map message arrives         ──► map_callback()    (once)
         │                                        saves the wall map
         │
         └─ the timer fires (20 per second)  ──► control_step()    (section 7)
                                                    for every robot: compute_robot()
                                                      ├─ find_neighbours()
                                                      ├─ separation_rule()
                                                      ├─ alignment_rule()
                                                      ├─ cohesion_rule()
                                                      ├─ compute_obstacle_acceleration()
                                                      ├─ combine (weighted sum or priority)
                                                      ├─ v = v + a * dt
                                                      └─ limit_speed()
                                                    then for every robot:
                                                      update_heading(), send_velocity(),
                                                      publish_accelerations(), write_log_row()
```

**Callbacks only store data. All the thinking happens in `control_step()`.**

---

## 5. Startup: `FlockingNode.__init__`

`__init__` runs once and has five parts.

**Part 1 – Read the parameters.**
Every value from `flocking_params.yaml` is read with `self.declare_parameter(name, default).value`.
If a value is missing from the YAML, the default written in the code is used.
`self.dt = 1.0 / self.control_rate` is the time between two control steps (0.05 s at 20 Hz).

**Part 2 – Create one `RobotState` per robot.**
`RobotState` (in `reynolds_rules.py`) is a simple box of numbers for one robot:

| Variable | Meaning |
|---|---|
| `x`, `y` | position (from odom) |
| `yaw` | body rotation (from odom, stays ≈ 0 because we never rotate) |
| `measured_vx`, `measured_vy` | actual velocity (from odom) |
| `command_vx`, `command_vy` | the velocity **we** command. Changed a bit every step |
| `heading` | direction of travel, used for the field of view |
| `has_odom` | becomes `True` after the first odom message |

**Part 3 – Publishers and subscribers.**
For every robot: one `cmd_vel` publisher, five acceleration publishers (for plots),
and one `odom` subscriber. All odom subscribers use the **same** callback function.
The callback finds out which robot a message belongs to from the message itself (section 6).
The `/map` subscriber uses a special QoS setting called "transient local". The map server
sends the map only once, and this setting lets a node that starts later still receive it.

**Part 4 – Start the timer.** `self.create_timer(self.dt, self.control_step)` means
"call `control_step` every 0.05 seconds".

---

## 6. Callbacks: storing the incoming data

### 6.1 `odom_callback(msg)`

**Which robot is it?** `msg.header.frame_id` is a text like `"robot_3/odom"`:

```python
robot_name = msg.header.frame_id.split('/')[0]          # "robot_3"
robot_id = int(robot_name.replace('robot_', ''))         # 3
```

**Position:** copied directly: `robot.x = msg.pose.pose.position.x`.
The simulator uses `localization "gps"`, so this is the true global position.

**Yaw from the quaternion.** ROS stores rotation as a quaternion $(x, y, z, w)$. The rotation
around the vertical axis is

$$\psi = \operatorname{atan2}\big(2(wz + xy),\; 1 - 2(y^2 + z^2)\big)$$

```python
robot.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
```

**Velocity: from the robot's frame to the world frame.** Stage reports the velocity as
seen by the robot itself (body frame). We rotate it by the yaw $\psi$:

$$
\begin{bmatrix} v_x^{world} \\ v_y^{world} \end{bmatrix} =
\begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix}
\begin{bmatrix} v_x^{body} \\ v_y^{body} \end{bmatrix}
$$

```python
robot.measured_vx = math.cos(robot.yaw) * body_vx - math.sin(robot.yaw) * body_vy
robot.measured_vy = math.sin(robot.yaw) * body_vx + math.cos(robot.yaw) * body_vy
```

We never send a rotation command, so $\psi \approx 0$, $\cos\psi = 1$, $\sin\psi = 0$ and nothing changes.
The rotation is there to keep the code correct even if a robot gets turned.

### 6.2 `map_callback(msg)`

This turns the ROS `OccupancyGrid` into an `ObstacleMap` object (section 8.6). This is the same
black-and-white bitmap that Stage uses for the walls.

---

## 7. The main loop: `control_step()`

Runs 20 times per second.

1. **Wait for data.** If any robot has `has_odom == False`, print
   "Waiting for odom..." and return.
2. **First time only: random start velocities.** The robots spawn standing still. A robot
   standing still has no direction, so its field of view would be undefined, and
   with zero velocities alignment has nothing to align to. `give_random_start_velocities()`
   gives each robot a velocity of size `initial_speed` in a random direction.
   The random generator uses a fixed `random_seed`, so **every run starts the same way**
   (lecture slide "Initial conditions": a repeatable random generator makes tuning easier).
3. **Phase 1 – compute.** `compute_robot(robot)` for every robot. The result is stored and
   nothing is sent yet.
4. **Phase 2 – apply.** For every robot: store the new velocity, update the heading, send
   `cmd_vel` and publish the accelerations.

**Why two phases?** All robots should decide based on the **same snapshot** of the world,
as if they all decided at the same moment. If robot 0's new velocity were applied before
robot 1 computes, robot 1 would react to information that real robot 1 could not have yet.

---

## 8. `compute_robot()` – all the math for one robot

This implements the formula from the lecture (slide "Combining accelerations"):

$$
\ddot{\boldsymbol{x}}_i =
\underbrace{\frac{c_a}{|\mathcal{N}_i|}\sum_{j\neq i}\dot{\boldsymbol{x}}_j - \dot{\boldsymbol{x}}_i}_{\text{alignment}}
+ \underbrace{\frac{c_c}{|\mathcal{N}_i|}\sum_{j\neq i}\boldsymbol{x}_j - \boldsymbol{x}_i}_{\text{cohesion}}
- \underbrace{\frac{c_s}{|\mathcal{N}_i|}\sum_{j\neq i}\frac{\boldsymbol{x}_j-\boldsymbol{x}_i}{\lVert\boldsymbol{x}_j-\boldsymbol{x}_i\rVert^2}}_{\text{separation}}
$$

plus an obstacle avoidance term. The symbols:

| Symbol | Meaning | In the code |
|---|---|---|
| $i$ | the robot we are computing for | `my_robot` / `robot` |
| $j$ | one neighbour | `other_robot` |
| $\mathcal{N}_i$ | the list of neighbours of robot $i$ | `neighbours` |
| $\lvert\mathcal{N}_i\rvert$ | how many neighbours | `number_of_neighbours = len(neighbours)` |
| $\boldsymbol{x}_i, \boldsymbol{x}_j$ | positions | `my_robot.x, my_robot.y`, `other_robot.x, other_robot.y` |
| $\dot{\boldsymbol{x}}_i$ | my velocity | `my_robot.command_vx, my_robot.command_vy` |
| $\dot{\boldsymbol{x}}_j$ | neighbour's velocity | `other_robot.measured_vx, other_robot.measured_vy` |
| $c_s, c_a, c_c$ | rule weights ("strength") | `weight_separation`, `weight_alignment`, `weight_cohesion` |
| $\ddot{\boldsymbol{x}}_i$ | resulting acceleration | `total_x, total_y` |

**How to read the slide's notation:** the $-\dot{\boldsymbol{x}}_i$ and $-\boldsymbol{x}_i$ are
**outside** the sum. "$\frac{1}{|\mathcal{N}_i|}\sum$" is the **average** over neighbours, so alignment
is (*average velocity − my velocity*) and cohesion is (*average position − my position*).
In the code the weight multiplies the whole difference: $c_a \cdot (\bar{v} - v_i)$.

Every vector is stored as two separate numbers, the x part and the y part. So every formula
appears **twice** in the code, once for x and once for y.

### 8.1 Step 1 – Who are my neighbours? `find_neighbours()` and `is_neighbour()`

The neighbourhood (lecture "Agent vision", lab Figure 2) is a circle of radius $R$
with a blind spot behind the robot. Robot $j$ is a neighbour of robot $i$ when **both** are true:

$$
d_{ij} = \lVert \boldsymbol{x}_j - \boldsymbol{x}_i \rVert \le R
\qquad\text{and}\qquad
\lvert \operatorname{wrap}(\theta_{ij} - h_i) \rvert \le \frac{\text{FOV}}{2}
$$

| Symbol | Meaning | Code |
|---|---|---|
| $d_{ij}$ | distance between the robots | `distance = vector_length(dx, dy)` |
| $R$ | neighbourhood radius | `neighbour_radius` |
| $\theta_{ij} = \operatorname{atan2}(y_j - y_i,\, x_j - x_i)$ | direction from me to the neighbour | `angle_to_other = math.atan2(dy, dx)` |
| $h_i$ | my heading (direction of my velocity) | `my_robot.heading` |
| $\operatorname{wrap}(\cdot)$ | bring the angle into $[-\pi, \pi]$ | `wrap_angle()` |
| FOV | field of view in degrees | `fov_degrees` |

```python
dx = other_robot.x - my_robot.x                 # x_j - x_i  (x part)
dy = other_robot.y - my_robot.y                 # x_j - x_i  (y part)
distance = vector_length(dx, dy)                # d_ij = sqrt(dx² + dy²)
if distance > neighbour_radius:                 # d_ij > R  -> too far
    return False
...
angle_to_other = math.atan2(dy, dx)             # θ_ij
angle_difference = wrap_angle(angle_to_other - my_robot.heading)   # wrap(θ_ij - h_i)
half_fov = math.radians(fov_degrees) / 2.0      # FOV / 2  (in radians)
if abs(angle_difference) <= half_fov:
    return True
```

**Why `wrap_angle`?** If the heading is 170° and the neighbour is at −170°, the raw
difference is −340°, but the real difference is only 20°. `wrap_angle` adds or subtracts
360° until the value is between −180° and +180°.

**Example:** FOV = 270° → half_fov = 135°. A neighbour exactly behind the robot has
angle_difference = 180° > 135°, so it is **not** seen. A neighbour at 90° (to the side) is seen.

`find_neighbours()` loops over all robots, skips the robot itself (`j ≠ i` in the formula),
and collects those for which `is_neighbour` is `True`. The result is $\mathcal{N}_i$.

### 8.2 Step 2a – Separation: `separation_rule()`

$$
\boldsymbol{a}_{sep} = -\frac{c_s}{|\mathcal{N}_i|}\sum_{j\in\mathcal{N}_i}\frac{\boldsymbol{x}_j-\boldsymbol{x}_i}{\lVert\boldsymbol{x}_j-\boldsymbol{x}_i\rVert^{p}}
$$

```python
for other_robot in neighbours:                       # Σ over j in N_i
    dx = other_robot.x - my_robot.x                  # (x_j - x_i), x part
    dy = other_robot.y - my_robot.y                  # (x_j - x_i), y part
    distance = vector_length(dx, dy)                 # ||x_j - x_i||
    if distance < 0.01:                              # protection: never divide by 0
        distance = 0.01
    divider = distance ** distance_power             # ||x_j - x_i||^p
    sum_x = sum_x + dx / divider                     # add one term of the sum
    sum_y = sum_y + dy / divider

acceleration_x = -weight_separation / number_of_neighbours * sum_x   # - c_s / |N_i| * Σ
acceleration_y = -weight_separation / number_of_neighbours * sum_y
```

**What each part does:**

- $(\boldsymbol{x}_j - \boldsymbol{x}_i)$ points from me **to** the neighbour. The **minus** in front
  flips it to point **away** from the neighbour. That is the repulsion.
- Dividing by $\lVert\cdot\rVert^p$ makes close neighbours push harder.
  The vector on top already has length $d$, so the push from one neighbour has size
  $d / d^p = 1/d^{\,p-1}$:
  - $p = 2$ (exactly the lecture formula) → push ∝ $1/d$
  - $p = 3$ (**our default**) → push ∝ $1/d^2$
- $c_s$ sets the overall strength. $\frac{1}{|\mathcal{N}_i|}$ turns the sum into an average.

**Why $p = 3$?** The lecture says repulsion in nature is "quadratic or cubic rather than linear".
In testing (section 11), $p = 2$ was too soft: while turning at a wall, robots squeezed to
9 cm apart, centre to centre (the robots are 7.5 cm wide). With $p = 3$ the smallest distance stayed
above 11 cm. Set `separation_distance_power: 2.0` to get exactly the slide formula.

**Example:** I am at (0, 0) and my only neighbour is at (0.2, 0). Then $c_s = 0.06$, $p = 3$:
dx = 0.2, distance = 0.2, divider = 0.008, sum_x = 0.2 / 0.008 = 25,
acceleration_x = −0.06 / 1 × 25 = **−1.5 m/s²**, a strong push to the left (away from the neighbour).
At distance 0.4 m it would be only −0.375 m/s² (4 times weaker for 2 times the distance).

### 8.3 Step 2b – Alignment: `alignment_rule()`

$$
\boldsymbol{a}_{ali} = c_a\left(\underbrace{\frac{1}{|\mathcal{N}_i|}\sum_{j\in\mathcal{N}_i}\dot{\boldsymbol{x}}_j}_{\bar{\boldsymbol{v}}\ \text{(average velocity)}} - \dot{\boldsymbol{x}}_i\right)
$$

```python
for other_robot in neighbours:                         # Σ over j
    sum_vx = sum_vx + other_robot.measured_vx          # + ẋ_j (x part)
    sum_vy = sum_vy + other_robot.measured_vy
average_vx = sum_vx / number_of_neighbours             # v̄ = Σ / |N_i|
average_vy = sum_vy / number_of_neighbours
acceleration_x = weight_alignment * (average_vx - my_robot.command_vx)   # c_a (v̄ - ẋ_i)
acceleration_y = weight_alignment * (average_vy - my_robot.command_vy)
```

**What each part does:**

- $\bar{\boldsymbol{v}} - \dot{\boldsymbol{x}}_i$ is "how different am I from the group". If I already move
  like my neighbours, it is 0 and alignment does nothing.
- $c_a$ is like a speed of reaction. With $c_a = 1$, about 63 % of the difference is
  corrected in 1 second.
- The neighbours' velocity is the **measured** one (from odom, what a robot could observe).
  My own velocity is the one I **command** (what the robot itself knows it is doing).

### 8.4 Step 2c – Cohesion: `cohesion_rule()`

$$
\boldsymbol{a}_{coh} = c_c\left(\underbrace{\frac{1}{|\mathcal{N}_i|}\sum_{j\in\mathcal{N}_i}\boldsymbol{x}_j}_{\text{local centre of mass}} - \boldsymbol{x}_i\right)
$$

```python
for other_robot in neighbours:                     # Σ over j
    sum_x = sum_x + other_robot.x                  # + x_j
    sum_y = sum_y + other_robot.y
centre_x = sum_x / number_of_neighbours            # local centre = Σ / |N_i|
centre_y = sum_y / number_of_neighbours
acceleration_x = weight_cohesion * (centre_x - my_robot.x)   # c_c (centre - x_i)
acceleration_y = weight_cohesion * (centre_y - my_robot.y)
```

**What each part does:**

- (centre − my position) points from me to the middle of the robots I can see.
  It is a **linear** pull: twice as far → twice as strong.
- A robot at the **edge** of the group has its centre far away → strong pull inwards.
  A robot in the **middle** has its centre close to itself → almost no pull.
  This matches the lecture: "border agent attracted inwards, inside agents more or less neutral".
- The centre is **local** (only the neighbours), not the centre of all robots. This allows the
  flock to split around an obstacle and join again (lecture: "having local attraction is better").

### 8.5 Separation vs. cohesion: why the robots keep a distance

Separation pushes away with $c_s/d^{p-1}$, cohesion pulls in with $c_c \cdot d$. Where they are
equal, the robots stop getting closer. For two robots alone:

$$c_c\,d = \frac{c_s}{d^{\,p-1}} \quad\Rightarrow\quad d = \left(\frac{c_s}{c_c}\right)^{1/p}$$

With $c_s = 0.06$, $c_c = 0.5$, $p = 3$: $d = (0.12)^{1/3} \approx 0.49$ m. With 10 robots, everyone
is pulled towards the middle by many neighbours, so the flock is packed tighter. The measured
average nearest-neighbour distance was **0.28 m**.

In the acceleration plot you can see this balance: in steady flight the separation curve and
the cohesion curve are equal in size and opposite in sign, so they cancel (the lecture's
"potential for component canceling"). The total is ≈ 0.

**Rule of thumb:** bigger $c_s$ or smaller $c_c$ gives a looser flock. Smaller $c_s$ or bigger $c_c$
gives a tighter flock (and with too small $c_s$ the robots collide).

### 8.6 Step 3 – Obstacle avoidance: `compute_obstacle_acceleration()`

The Stage robots have no distance sensors, so we **emulate perception**: we read the
map and check whether a point in front of the robot is inside a wall.

#### The map: `ObstacleMap`

The map is a grid of 5 cm × 5 cm cells (200 × 200 for the 10×10 map). `is_wall(x, y)` turns
a world point into a cell:

$$\text{column} = \left\lfloor \frac{x - x_{origin}}{\text{resolution}} \right\rfloor,\qquad
\text{row} = \left\lfloor \frac{y - y_{origin}}{\text{resolution}} \right\rfloor,\qquad
\text{index} = \text{row}\cdot\text{width} + \text{column}$$

```python
column = int(math.floor((x - self.origin_x) / self.resolution))
row = int(math.floor((y - self.origin_y) / self.resolution))
index = row * self.width + column        # the grid is stored as one long list, row after row
value = self.cells[index]                # 100 = wall, 0 = free
```

Points outside the map count as walls.

Three modes are available (`obstacle_mode`): `'steer'` (default), `'field'` and `'none'`.

#### Mode `'steer'` – Steer to avoid (lecture slide with circles A, B, C)

**a) Look ahead through a tube: `free_distance()`.** From the robot, walk forward along the
heading in 5 cm steps up to `look_ahead` (0.8 m). At every step, check three points:
the centre line and two side lines `clearance` (0.15 m) to the left and right. This is the
yellow tube in the slide picture: the path enlarged by the clearance size. The first
distance where any point is a wall is returned. If nothing is hit, `look_ahead` is returned.

```python
direction_x = math.cos(direction_angle)       # unit vector along the path
direction_y = math.sin(direction_angle)
side_x = -direction_y                         # unit vector to the left (rotated by 90°)
side_y = direction_x
...
point_x = x + direction_x * distance + side_x * side_offset   # point = robot + forward + sideways
```

The tube is narrower near the robot (it grows from 0 to `clearance`, like a pencil tip).
Otherwise a robot driving along a wall would see the wall in **every** direction and could
not choose where to go. Walls right next to the robot are handled by the touch sensor.

**b) Is anything in front of me?** If `distance_ahead >= look_ahead`, the tube is free and
the result is **0**. Obstacles beside or behind the robot have no effect. This is the
"perception-based" advantage over the force field.

**c) Find the silhouette edge.** Try directions turned 20°, 40°, 60° … 180° to the left and
right. The first direction whose tube is completely free is the closest way around the obstacle.
This is the 2D version of "steer towards the silhouette edge, enlarged by the clearance".
If no direction is completely free, the one with the most free space is used.

**d) Steer.** With $\hat{\boldsymbol{t}}$ = unit vector of the free direction and
$\hat{\boldsymbol{h}}$ = unit vector of the current heading:

$$
\boldsymbol{a}_{obs} = c_o \cdot u \cdot \frac{\hat{\boldsymbol{t}} - \hat{\boldsymbol{h}}}{\lVert\hat{\boldsymbol{t}} - \hat{\boldsymbol{h}}\rVert},
\qquad u = \left(1 - \frac{d_{ahead}}{L}\right)^2
$$

| Symbol | Meaning | Code |
|---|---|---|
| $c_o$ | obstacle weight | `weight_obstacle` |
| $d_{ahead}$ | free distance straight ahead | `distance_ahead` |
| $L$ | look-ahead distance | `look_ahead` |
| $u$ | urgency, 0 (far) … 1 (touching) | `urgency = closeness * closeness` |
| $\hat{\boldsymbol{t}} - \hat{\boldsymbol{h}}$ | "change of direction" I want | `steer_x, steer_y` |

```python
closeness = 1.0 - distance_ahead / look_ahead
urgency = closeness * closeness                            # u
steer_x = math.cos(best_angle) - math.cos(heading)         # t̂ - ĥ (x part)
steer_y = math.sin(best_angle) - math.sin(heading)
steer_length = vector_length(steer_x, steer_y)             # ||t̂ - ĥ||
acceleration_x = weight_obstacle * urgency * steer_x / steer_length
```

$\hat{\boldsymbol{t}} - \hat{\boldsymbol{h}}$ points mostly **sideways** for small turns, so the robot
**steers around** the obstacle instead of only braking.

Urgency is **squared**. A linear urgency was tested first. With it, the acceleration
jumped on and off every step when a wall was just at the tip of the tube, because a small turn
cleared the tube and alignment turned the robot back. Squaring makes far walls give almost no
push (no flicker) and close walls give a strong push.

**e) Touch sensor: `touch_sensor()`.** If any wall cell is closer than `touch_distance` (0.2 m),
push straight away from the nearest wall point:

$$\boldsymbol{a}_{touch} = c_o\left(1 - \frac{d_{wall}}{d_{touch}}\right)\frac{\boldsymbol{x}_i - \boldsymbol{x}_{wall}}{d_{wall}}$$

`nearest_wall_point()` only searches the small square around the robot, so it is fast.
In `'steer'` mode, the obstacle acceleration = steer + touch.

#### Mode `'field'` – Force field (lecture slide with the bump-shaped curve)

The nearest wall point (searched up to $3w$) pushes the robot away. The strength follows the
curve from the slide:

$$s(d) = c_o\cdot\frac{d}{w}\cdot e^{\frac{1}{2}\left(1 - (d/w)^2\right)}$$

It is 0 at the wall centre, has its maximum $c_o$ at $d = w$ (`field_width` = 0.25 m), and fades
out further away.

```python
ratio = distance / field_width                                        # d / w
strength = weight_obstacle * ratio * math.exp(0.5 * (1.0 - ratio * ratio))
return strength * away_x, strength * away_y                           # direction: away from the wall
```

This shows both problems listed on the slide. The push acts all around the robot, including
walls it already passed. And when driving straight at a wall, it only pushes **backwards**:
the robot slows down instead of steering away. In the test, the whole flock **reversed**
at the corner instead of turning along the wall (section 11).

### 8.7 Step 4 – Combining the accelerations

**`combine_mode: 'weighted_sum'` (default, the lecture's "strength parameter" method):**

$$\boldsymbol{a} = \operatorname{limit}\big(\boldsymbol{a}_{sep} + \boldsymbol{a}_{ali} + \boldsymbol{a}_{coh} + \boldsymbol{a}_{obs},\; a_{max}\big)$$

```python
total_x = separation_x + alignment_x + cohesion_x + obstacle_x
total_y = separation_y + alignment_y + cohesion_y + obstacle_y
total_x, total_y = limit_vector(total_x, total_y, self.max_acceleration)
```

The weights $c_s, c_a, c_c, c_o$ are already inside each rule. `limit_vector` shrinks the vector
to length $a_{max}$ if it is longer. The direction stays the same. A real robot's motors
cannot accelerate without limit.

**`combine_mode: 'priority'` (lecture "prioritized acceleration allocation"):**
A budget of $a_{max}$ is shared in the order **obstacle → separation → alignment → cohesion**.
Each request takes as much of the budget as it needs. When the budget runs out, the current
request is cut to what is left and the rest are ignored.

```python
for acceleration in accelerations_in_priority_order:
    size = vector_length(acceleration_x, acceleration_y)
    if size <= budget_left:                       # fits completely
        total_x = total_x + acceleration_x
        budget_left = budget_left - size
    else:                                         # only a part fits
        fraction = budget_left / size
        total_x = total_x + acceleration_x * fraction
        break                                     # lower priorities get nothing
```

So when a wall is close, obstacle avoidance gets the full acceleration and cohesion gets none.
That avoids the "indecision" where two rules cancel each other out.

### 8.8 Step 5 – From acceleration to velocity (the "real robot" rule)

The lab says: *do not command big jumps in velocity, compute an acceleration and update
the velocity gradually.* This is the explicit Euler integration:

$$\boldsymbol{v}_{new} = \boldsymbol{v}_{old} + \boldsymbol{a}\,\Delta t$$

```python
new_vx = robot.command_vx + total_x * self.dt     # v_new = v_old + a · Δt   (Δt = 0.05 s)
new_vy = robot.command_vy + total_y * self.dt
```

With $a_{max} = 0.5$ m/s² and $\Delta t = 0.05$ s, the velocity can change by at most **2.5 cm/s
per step**. The velocity plots therefore show smooth curves, never steps.

**`limit_speed()`** then keeps the speed $\lVert\boldsymbol{v}\rVert$ between `min_speed` and `max_speed`:

- faster than `max_speed` → scale down to `max_speed`, same direction,
- slower than `min_speed` → scale up to `min_speed`, same direction. If the velocity is exactly 0, the
  robot goes along its last heading.

A minimum speed is part of Reynolds' boid model (birds cannot hover). Without it, alignment
averages the random start directions, which mostly cancel out, and the flock almost stops.

### 8.9 After computing: heading, sending, logging

**`update_heading()`.** The robots are omnidirectional, so the heading is the direction of the
velocity (lab footnote 1):

$$h_i = \operatorname{atan2}(v_y, v_x)$$

It is only updated if the speed is above 1 cm/s. A standing robot has no direction, so the
old heading is kept.

**`send_velocity()`.** Stage wants the velocity in the robot's own frame, so we rotate the
world velocity by $-\psi$ (the inverse of section 6.1):

$$
\begin{bmatrix} v_x^{body} \\ v_y^{body} \end{bmatrix} =
\begin{bmatrix} \cos\psi & \sin\psi \\ -\sin\psi & \cos\psi \end{bmatrix}
\begin{bmatrix} v_x^{world} \\ v_y^{world} \end{bmatrix}
$$

`angular.z` is always 0, so the robots never rotate and $\psi$ stays 0.

**`publish_accelerations()`** sends the five acceleration vectors on the plotting topics, so
`ros2 bag` can record them.

---

## 9. Shutting down

When you press `Ctrl+C`, `rclpy.spin` stops, and if ROS is still alive
a zero velocity is sent to every robot. If a `Ctrl+C` arrives in the middle of a control step,
`control_step` checks `rclpy.ok()` before each publish and simply returns, instead of crashing
with "publisher's context is invalid".

---

## 10. The plots (`plot_flocking.py`)

`plot_flocking.py` reads a ROS 2 bag and draws three pictures:

| File | Lab requirement |
|---|---|
| `velocities_all_robots.png` | 4a – commanded vs. actual $v_x$ and $v_y$ of **every** robot over time |
| `velocity_robot_X.png` | 4a – commanded vs. actual $v_x$ and $v_y$ of one robot |
| `accelerations_robot_X.png` | 4b – separation, alignment, cohesion, obstacle and **total** acceleration of one robot (x, y and size) |

**Where the data comes from:**

| Line in the plot | Topic in the bag |
|---|---|
| commanded velocity | `/robot_X/cmd_vel` (rotated to the world frame) |
| actual velocity | position in `/robot_X/odom` |
| rule accelerations | `/robot_X/flocking/separation`, `alignment`, `cohesion`, `obstacle` |
| total acceleration | `/robot_X/flocking/total_acceleration` |

**Why actual velocity comes from the position.** Stage copies the commanded velocity into the
twist part of `/robot_X/odom`, even when a robot is blocked and not moving. So the twist can never
show a collision. The script compares each position with the position 20 messages later (0.2 s):

$$
\boldsymbol{v}_{actual} = \frac{\boldsymbol{x}(t + 0.2) - \boldsymbol{x}(t)}{0.2}
$$

Free robots have an actual velocity almost equal to the command, because Stage has no inertia.
A blocked robot shows an actual velocity near zero while the command stays high.

---

## 11. Test results (how the default parameters were chosen)

These are the first tuning runs. Each row is one run, so the numbers change a little between
repeats. Later repeats of the row 5 settings had a brief contact at about 7 s.

All runs: 10 robots, `empty_10x10` (unless stated), seed 42, `neighbour_radius` 1.0 m.
"Wall" = closest distance from a robot centre to the wall surface.

| Run | What changed | Order | Avg NN dist. | Smallest dist. | Wall | Observation |
|---|---|---|---|---|---|---|
| 1 | $c_s$=0.05, $p$=2, min speed 0.1 | 1.00 | 0.16 m | **0.075 m** | – | Flock forms but robots **touch**. Slow (0.1 m/s), never reached a wall in 60 s |
| 2 | $c_s$=0.15, min speed 0.15 | 1.00 | 0.28 m | 0.15 m | – | Good spacing, but the obstacle acceleration **flickered** on/off at the wall |
| 3 | + squared urgency | 0.99 | 0.27 m | 0.09 m | 0.17 m | Flicker gone, but the flock **compressed** to 9 cm during the wall turn |
| 4 | $p$=3, $c_s$=0.04 | 1.00 | 0.25 m | 0.10 m | 0.17 m | Stiffer separation, turn is smoother |
| **5** | **$p$=3, $c_s$=0.06 (final default)** | **1.00** | **0.28 m** | **0.12 m** | **0.18 m** | **One aligned flock, no contact in this run, smooth wall turns** |
| 6 | FOV 270° | 0.99 | 0.30 m | 0.09 m | 0.18 m | Sees only 5.7 neighbours instead of 9, still flocks, robots get closer (cannot see behind) |
| 7 | `obstacle_mode: field` | 1.00 | 0.28 m | 0.12 m | 0.50 m | Flock **reversed** at the corner instead of turning (the slide's "reduces movement rather than steering") |
| 8 | `combine_mode: priority` | 1.00 | 0.28 m | 0.11 m | 0.16 m | Same as the weighted sum in open space, works |
| 9 | map `empty_3x3` | 0.99 | 0.26 m | 0.14 m | 0.17 m | Flock circles inside the small arena, turning at every wall |

---

## 12. Parameter cheat-sheet (what happens if you change it)

The effects of `weight_separation`, `separation_distance_power`, `min_speed`, `fov_degrees`,
`obstacle_mode`, `combine_mode` and the map size were measured (section 11). The other rows are
the **expected** behaviour, so check them in your own runs before writing them in the report.

| Parameter | Increase → | Decrease → |
|---|---|---|
| `weight_separation` ($c_s$) | looser flock, larger gaps | tighter flock, below ≈ 0.05 robots touch |
| `weight_cohesion` ($c_c$) | tighter, more "elastic" flock | flock spreads out, may split into groups |
| `weight_alignment` ($c_a$) | robots agree on a direction faster, very ordered | slow to agree, more random-looking motion |
| `separation_distance_power` ($p$) | push grows faster when close (stiffer, safer) | softer push, more squeezing in turns |
| `neighbour_radius` ($R$) | each robot sees more, one big flock | sees fewer robots, flock can break into groups |
| `fov_degrees` | sees more robots, more stable | blind spot behind, fewer neighbours, robots behind can come close |
| `max_acceleration` | quicker reactions, sharper turns | smoother but slower reactions, can be too slow to avoid walls |
| `min_speed` / `max_speed` | faster flock | slower flock |
| `look_ahead` | turns away from walls earlier, wider turns | turns later, closer to the walls |
| `clearance` | keeps more distance from walls | passes closer to walls |
| `weight_obstacle` | stronger and earlier wall avoidance | weaker, may touch the walls |
| `num_robots` (in `launch_params.yaml`) | more neighbours, denser flock, slower computation (O(n²)) | fewer neighbours, flock less stable |
| `obstacle_mode` | `steer`: turns along walls · `field`: bounces/reverses · `none`: robots get stuck on walls | |
| `combine_mode` | `priority`: obstacle and separation always win when it matters · `weighted_sum`: everything mixed, rules can cancel each other | |
