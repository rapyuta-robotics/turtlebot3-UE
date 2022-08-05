#!/bin/bash
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

CURRENT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
TB3_UE_DIR=${1:-"${CURRENT_DIR}"}

#change default level, generating DefautlEngine.ini
DEFAULT_LEVEL=${LEVEL_NAME:-"Turtlebot3_benchmark"}
sed -e 's/${LEVEL_NAME}/'${DEFAULT_LEVEL}'/g' Config/DefaultEngineBase.ini > Config/DefaultEngine.ini

UE_EDITOR=${2:-"$(dirname ${TB3_UE_DIR})/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"}
(exec "$UE_EDITOR" "${TB3_UE_DIR}/turtlebot3.uproject" "-game")
