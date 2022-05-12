#!/bin/bash

# Check ROS env vars
(printenv | grep -i ROS) || echo '[Error] Found nothing with ROS in env variables'
export ROS_DOMAIN_ID=10

# Check [launch_testing] pkg
echo "launch_testing pkg:"
ros2 pkg prefix launch_testing
echo "launch_testing_ros pkg:"
ros2 pkg prefix launch_testing_ros

# Create a workspace to build required dependency ros2 pkgs
BASE_DIR="$(pwd)"
RRSIM_ROS2_WS="${BASE_DIR}/RRSIM_ROS2"

# Install [ue_msgs] pkg
cd $BASE_DIR
echo "Installing ue_msgs..."
if [ ! -e UE_msgs ]; then
	echo "Cloning UE_msgs..."
	git clone https://github.com/rapyuta-robotics/UE_msgs.git
else
	echo "Updating UE_msgs..."
	cd UE_msgs
	git pull --rebase origin devel
fi

if [ ! -e ${RRSIM_ROS2_WS} ]; then
	mkdir -p ${RRSIM_ROS2_WS}/src
	cd ${RRSIM_ROS2_WS}/src
	ln -s ${BASE_DIR}/UE_msgs UE_msgs
fi

cd ${RRSIM_ROS2_WS}
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix ue_msgs
cd $BASE_DIR
