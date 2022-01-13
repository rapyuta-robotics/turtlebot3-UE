#!/bin/bash
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
CURRENT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
TB3_UE_DIR=${1:-"${CURRENT_DIR}"}
RCLUE_DIR="${TB3_UE_DIR}/Plugins/rclUE"
source ${RCLUE_DIR}/Scripts/setup_ros2libs.sh ${RCLUE_DIR}
UE_EDITOR=${2:-"$(dirname ${TB3_UE_DIR})/UnrealEngine/Engine/Binaries/Linux/UE4Editor"}
(exec "$UE_EDITOR" "${TB3_UE_DIR}/turtlebot3.uproject" "-game")