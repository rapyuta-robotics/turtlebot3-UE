#!/bin/sh

#export ROS_MASTER_URI=http://localhost:11311

#unset ROS_DOMAIN_ID
EDITOR_COMMAND="../UnrealEngine/Engine/Binaries/Linux/UE4Editor"

(exec "$EDITOR_COMMAND" "${PWD}/turtlebot3.uproject" "-game")
