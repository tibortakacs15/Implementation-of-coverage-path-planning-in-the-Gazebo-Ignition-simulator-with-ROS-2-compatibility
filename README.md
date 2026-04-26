## ▶️ Run Instructions (ROS 2 Pipeline)

## BUILD
```bash
cd ~/coverage_planner_ws
colcon build
source install/setup.bash

TERMINAL 1 – ACTION SERVER
cd ~/coverage_planner_ws
source install/setup.bash

ros2 run coverage_planner coverage_action_server

TERMINAL 2 – PATH VISUALIZER (PARAMETER!)
cd ~/coverage_planner_ws
source install/setup.bash

ros2 run coverage_planner coverage_path_visualizer \
--ros-args -p map_name:=living_room.png

TERMINAL3 – MAP SERVER
cd ~/coverage_planner_ws
source install/setup.bash

ros2 run nav2_map_server map_server \
--ros-args -p yaml_filename:=/home/takacs/coverage_planner_ws/maps/living_room.yaml

TERMINAL 4 – LIFECYCLE
cd ~/coverage_planner_ws source install/setup.bash

ros2 lifecycle set /map_server configure

ros2 lifecycle set /map_server activate

TERMINAL 5 – RViz
cd ~/coverage_planner_ws source
install/setup.bash
rviz2 
