#!/bin/sh

export LD_LIBRARY_PATH=\
"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib":\
"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib/std_msgs/lib":\
"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib/rosgraph_msgs/lib":\
"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib/geometry_msgs/lib":\
"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib/builtin_interfaces/lib":\
"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib/ue4_interfaces/lib"

make --file="./turtlebot3/Makefile" turtlebot3Editor
