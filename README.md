# Turtlebot UE
UE4 Project which includes examples to use rclUE.
## Maps
- PubSub: BP and C++ ROS2 publisher subscriber example.
- Turtlebot3 Benchmark: BP and C++ ROS2 turtlebot3 navigation. Burger is implemented in C++ and Waffle is implemented in BP
## Setup
1. clone this repo
2. ./run_editor.sh

## Turtlebot3 navigation
### Installation
1. [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
2. [Install Nav2](https://navigation.ros.org/getting_started/index.html)

### Run
1. Play turtlebot3-UE project
2. `ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False map:=<path to turtlebot3-UE>/Content/Turtlebot3_benchmark.yaml
`
