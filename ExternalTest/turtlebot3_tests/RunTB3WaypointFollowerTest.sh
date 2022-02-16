#!/bin/sh
mkdir maps
if [ ! -f maps/Turtlebot3_benchmark.pgm ]; then
    ln ../../Content/Turtlebot3_benchmark.pgm maps/Turtlebot3_benchmark.pgm
fi
if [ ! -f maps/Turtlebot3_benchmark.yaml ]; then
    ln ../../Content/Turtlebot3_benchmark.yaml maps/Turtlebot3_benchmark.yaml
fi

launch_test turtlebot3_tests/test_waypoint_follower.py tb3_model:=$1
