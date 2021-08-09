#!/bin/sh

export LD_LIBRARY_PATH=\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/std_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/geometry_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/sensor_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/rosgraph_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/nav_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/tf2_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/builtin_interfaces/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/ue4_interfaces/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/ue_msgs/lib"

#export ROS_MASTER_URI=http://localhost:11311

unset ROS_DOMAIN_ID
EDITOR_COMMAND="../UnrealEngine/Engine/Binaries/Linux/UE4Editor"

(exec "$EDITOR_COMMAND" "${PWD}/turtlebot3.uproject" "-game")
