#!/bin/bash
# Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

TB3_UE_DIR=$1
TB3_UE_PACKAGE=$2
TARGET_LEVEL=${LEVEL_NAME:-"/Game/Maps/Turtlebot3_benchmark.Turtlebot3_benchmark"}

# Run discovery service for FastDDS
(exec "${TB3_UE_DIR}/run_discovery_service.sh")

# Configure environment for FastDDS discovery
source ${TB3_UE_DIR}/fastdds_setup.sh

(exec "${TB3_UE_PACKAGE}/Linux/turtlebot3-UE.sh" "${TARGET_LEVEL}" "-Windowed" "-ResX=1280" "-ResY=720")
# OR
# To start [Entry], the default map as configured in DefaultEngine.ini, then [TARGET_LEVEL]
#(exec "${TB3_UE_PACKAGE}/Linux/turtlebot3-UE.sh" "-Windowed" "-ResX=1280" "-ResY=720" "-targetmap=${TARGET_LEVEL}")
#(exec "${TB3_UE_PACKAGE}/Linux/turtlebot3-UE.sh" "-Windowed" "-ResX=1280" "-ResY=720" "-targetmap=${TARGET_LEVEL}" "-targetRTF=${TARGET_RTF}" "-timestep=${TARGET_TIME_STEP}" )

pkill -P $$