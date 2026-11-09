## Running the simulation

With ROS 2 Jazzy installed, open a terminal and run:

```bash
source /opt/ros/jazzy/setup.bash &&
cd ~/coverage_planner_ws &&
colcon build --symlink-install --packages-up-to coverage_planner &&
source install/setup.bash &&
ros2 launch coverage_planner coverage_simulation.launch.py
```

The launch file reads the settings from `initialization.txt` and automatically
generates a Gazebo world for the selected map in the `world_sdf/` directory.

To make the robot follow the planned route, set `start_follower: true` in
`initialization.txt` before launching the simulation.
