#!/bin/sh
mkdir maps
if [ ! -f maps/Turtlebot3_benchmark.pgm ]; then
    ln ../../Content/Turtlebot3_benchmark.pgm maps/Turtlebot3_benchmark.pgm
fi
if [ ! -f maps/Turtlebot3_benchmark.yaml ]; then
    ln ../../Content/Turtlebot3_benchmark.yaml maps/Turtlebot3_benchmark.yaml
fi

launch_test turtlebot3_tests/test_waypoint_follower.py tb3_model:=$1\
                                                       waypoints:='-0.52, -0.78, 0.7, 0.5, 2.0, -1.5, 1.7, 1.7'\
                                                       initial_pose:='-1.0, 0.0, 0.0, 0.0, 0.0, 0.0'
