# Basic information
Turtlebot3 simulation with Unreal Engine4 with [rclUE](https://github.com/rapyuta-robotics/rclUE)
# Setup
## Ubuntu 20.04
### UE4 setting
1. Build UE4 by following tutorial [ref](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html)
2. copy rclUE under turtlebot3/Plugins/
2. Build and run editor
```
./make_editor.sh
./run_editor.sh
```
### ROS2 foxy setting
1. ros2 installation by following tutorial [ref](https://docs.ros.org/en/foxy/Installation/Linux-Development-Setup.html)
2. create ws and build
```
mkdir -p turtlebot3_ws/src
cd turtlebot3_ws/src
git clone git@github.com:rapyuta-robotics/turtlebot3-UE.git
source /opt/ros/foxy/setup.bash
cd .. 
rosdep install --from-paths . --ignore-src -y
colcon build
```
3. launch navigation with gazebo(to test navigation work properly)
```
ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=True
```
4. launch navigation with gazebo
```
ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False
```
## todo 