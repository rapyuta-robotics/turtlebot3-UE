#!/bin/sh
mkdir maps
ln ../../Content/Turtlebot3_benchmark.pgm maps/Turtlebot3_benchmark.pgm
ln ../../Content/Turtlebot3_benchmark.yaml maps/Turtlebot3_benchmark.yaml
launch_test src/test_waypoint_follower.py tb3_model:=burger
