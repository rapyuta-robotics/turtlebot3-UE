#!/bin/sh

GENERATOR_COMMAND="../UnrealEngine/GenerateProjectFiles.sh"

(exec "$GENERATOR_COMMAND" -project="${PWD}/turtlebot3/turtlebot3.uproject" -game)
