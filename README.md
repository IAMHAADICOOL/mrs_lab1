# MRS Lab 1: Reynolds Flocking

Ten simulated robots move together as one flock and avoid walls, using Reynolds' rules: separation, alignment and cohesion.

The simulator is Stage. The robot control code is in `src/reynolds_flocking`.

## What you need

- Ubuntu 24.04
- ROS 2 Jazzy (desktop install)

## Set up the workspace

Do these steps once.

**1. Download the code.** The `--recurse-submodules` part is needed, because it also downloads the `Stage` and `stage_ros2` folders.

```bash
git clone --recurse-submodules https://github.com/IAMHAADICOOL/mrs_lab1.git
cd mrs_lab1
```

**2. Install the required packages.**

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-ackermann-msgs
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the workspace.**

```bash
colcon build --symlink-install
```

## Run the lab

You need three terminals. Run these commands from the workspace folder.

**Terminal 1: start the simulator.** A Stage window should open.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sphero_stage start.launch.py
```

**Terminal 2: start recording a bag.**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch reynolds_flocking record_bag.launch.py bag_name:=my_test
```

`my_test` is the name of this run. The bag is saved in `flocking_logs/bags/my_test`. The bag is a folder, and that folder must not exist yet, so use a new name for each run.

It records these topics for every robot:

- `cmd_vel`: the commanded velocity
- `odom`: the position, used to work out the actual velocity
- `flocking/...`: the acceleration of each rule and the total

**Terminal 3: start the flocking.** The robots should start moving.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch reynolds_flocking flocking.launch.py
```

**To stop:** press `Ctrl+C` in Terminal 3, then Terminal 2, then Terminal 1. Always stop the recording with `Ctrl+C`, because the bag is only finished when the recorder closes it.

## See the results

After stopping, plot the bag:

```bash
ros2 run reynolds_flocking plot_flocking --bag_name my_test
```

This saves three plots in `flocking_logs/plots/my_test`:

- `velocities_all_robots.png`: commanded and actual vx and vy of every robot over time
- `velocity_robot_0.png`: commanded and actual velocity of one robot, in x and y
- `accelerations_robot_0.png`: the acceleration of each rule and the total, for one robot

You can also choose the robot and where to save the plots:

```bash
ros2 run reynolds_flocking plot_flocking --bag_name my_test --robot 3 --plot_folder ~/my_plots
```

The plot folder is created if it does not exist. Plots already in that folder are replaced.

## Change the settings

| What to change | File |
|---|---|
| Map, number of robots, start positions | `src/sphero_simulation/sphero_stage/launch/launch_params.yaml` |
| Flocking strengths, speeds, obstacle avoidance | `src/reynolds_flocking/config/flocking_params.yaml` |
| Where bags and plots are saved (`log_folder`) | `src/reynolds_flocking/config/flocking_params.yaml` |

You do not need to build again after editing these files. Just stop and start the launch commands again.

`log_folder` is a full path, so change it if your workspace is in a different place.

Maps you can use for `map_name`:

- `empty_10x10`
- `empty_3x3`
- `test_10x10`
- `random_10x10`
- `simple_maze_10x10`
- `hard_maze_10x10`
