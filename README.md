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

You need two terminals. Run these commands from the workspace folder.

**Terminal 1: start the simulator.** A Stage window should open.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sphero_stage start.launch.py
```

**Terminal 2: start the flocking.** The robots should start moving.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch reynolds_flocking flocking.launch.py run_label:=my_test
```

`my_test` is the name of this run. It is used for the log file, so you can pick any name.

**To stop:** press `Ctrl+C` in Terminal 2 first, then in Terminal 1.

## See the results

After stopping, run this in Terminal 2. It uses the newest log in `~/flocking_logs`.

```bash
ros2 run reynolds_flocking plot_flocking
```

It prints a summary of the run and saves the plots. By default, logs and plots are saved in `~/flocking_logs`.

You can also choose which log to plot and where to save the plots:

| Option | What it does |
|---|---|
| `--csv_file` | The log file to plot |
| `--plot_folder` | The folder where the plots are saved |

For example:

```bash
ros2 run reynolds_flocking plot_flocking --csv_file ~/flocking_logs/my_test_20261029_101500.csv --plot_folder ~/my_plots
```

To see the names of your log files, run `ls ~/flocking_logs/*.csv`.

The plot folder is created if it does not exist. Use a different folder for each run, because plots already in that folder are replaced.

## Change the settings

| What to change | File |
|---|---|
| Map, number of robots, start positions | `src/sphero_simulation/sphero_stage/launch/launch_params.yaml` |
| Flocking strengths, speeds, obstacle avoidance | `src/reynolds_flocking/config/flocking_params.yaml` |

You do not need to build again after editing these files. Just stop and start the launch commands again.

Maps you can use for `map_name`:

- `empty_10x10`
- `empty_3x3`
- `test_10x10`
- `random_10x10`
- `simple_maze_10x10`
- `hard_maze_10x10`
