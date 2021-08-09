#!/bin/sh

PROJECT_PATH="/home/yu/UnrealProject"
PROJECT_NAME="MyProject"

export LD_LIBRARY_PATH="$PROJECT_PATH/$PROJECT_NAME/Plugins/rclUE/Source/ThirdParty/ros2lib":"$PROJECT_PATH/$PROJECT_NAME/Plugins/rclUE/Source/ThirdParty/ros2lib/std_msgs/lib":"$PROJECT_PATH/$PROJECT_NAME/Plugins/rclUE/Source/ThirdParty/ros2lib/builtin_interfaces/lib/":"$PROJECT_PATH/$PROJECT_NAME/Plugins/rclUE/Source/ThirdParty/ros2lib/ue4_interfaces/lib/"


EDITOR_COMMAND="./ue"

(exec "$EDITOR_COMMAND" "$PROJECT_PATH/$PROJECT_NAME/$PROJECT_NAME.uproject")
