#!/bin/bash
# Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

if [ -z "${UE5_DIR}" ]; then
        printf "Please set UE5_DIR to path of UE5 UnrealEngine's parent folder\n"
        exit 1
fi

CURRENT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
TB3_UE_DIR=${1:-"${CURRENT_DIR}"}

# Run discovery service for FastDDS 
(exec "${TB3_UE_DIR}/run_discovery_service.sh")

# Configure environment for FastDDS discovery
source ${TB3_UE_DIR}/fastdds_setup.sh

UE_EDITOR="${UE5_DIR}/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"
(exec "$UE_EDITOR" "${TB3_UE_DIR}/turtlebot3.uproject" "-game")
