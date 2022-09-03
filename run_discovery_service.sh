#!/bin/bash

source fastdds_setup.sh
source /opt/ros/foxy/setup.bash
fastdds discovery -i 0 &
ros2 daemon start

