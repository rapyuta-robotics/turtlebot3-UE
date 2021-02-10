#!/bin/sh

export LD_LIBRARY_PATH="${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib":"${PWD}/turtlebot3/Plugins/rclUE/Source/ThirdParty/ros2lib/std_msgs/lib"

EDITOR_COMMAND="../UnrealEngine/Engine/Binaries/Linux/UE4Editor"

(exec "$EDITOR_COMMAND" "${PWD}/turtlebot3/turtlebot3.uproject")
