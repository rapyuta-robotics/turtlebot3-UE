# Turtlebot UE
UE4 Project which includes examples to use rclUE.
## Maps
- PubSub: BP and C++ ROS2 publisher subscriber example.
- Turltebot3 Benchmark: BP and C++ ROS2 turtleobot3 navigation. Burger is implemented in C++ and Waffle is implemented in BP

## Setup for Ubuntu 20.04 & ROS2 foxy

Retrieve the repository with all required submodules:
```
git clone --recurse-submodules git@github.com:rapyuta-robotics/turtlebot3-UE.git
```
Generate VSCode build configurations:
```
cd turtlebot3-UE
./update_project_files.sh
```
Beware that the update_project_files script considers that Unreal Engine and turtlebot3-UE are located in the same folder. If that's not the case, make a copy of the script with your correct Unreal Engine folder.

If the script fails because of obsolete temporary files, remove them:
```
rm -r Binaries Intermediate Saved Plugins/rclUE/Binaries Plugins/rclUE/Intermediate Plugins/RapyutaSimulationPlugins/Binaries Plugins/RapyutaSimulationPlugins/Intermediate
```
Build the plugins and UE4 project:
```
make turtlebot3Editor
```
Building can also be done inside VSCode:
- Launch VSCode and open folder "turtlebot3"
- Press Ctrl+Shift+B and choose "turtlebot3Editor Linux Development Build"

To run turtlebot3-UE project in Unreal Engine editor:
```
./run_editor.sh
```

## Turtlebot3 navigation
### Installation
1. [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
2. [Install Nav2](https://navigation.ros.org/getting_started/index.html)

### Run
1. Play turtlebot3-UE project
2. `ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False map:=<path to turtlebot3-UE>/Content/Turtlebot3_benchmark.yaml
`
