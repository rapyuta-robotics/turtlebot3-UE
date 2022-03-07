#!/bin/bash
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#source /home/cconti/Rapyuta/UnrealEngine/Engine/Build/BatchFiles/Linux/SetupEnvironment.sh -mono /home/cconti/Rapyuta/UnrealEngine/Engine/Build/BatchFiles/Linux;xbuild /property:Configuration=Development /verbosity:quiet /nologo /p:NoWarn=1591 /home/cconti/Rapyuta/UnrealEngine/Engine/Source/Programs/UnrealBuildTool/UnrealBuildTool.csproj
#mono /home/cconti/Rapyuta/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool.exe -ModuleWithSuffix=turtlebot3,2890 turtlebot3Editor Linux Development -Project="/home/cconti/Rapyuta/turtlebot3/turtlebot3.uproject" "/home/cconti/Rapyuta/turtlebot3/turtlebot3.uproject"  -IgnoreJunk -progress

CURRENT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
TB3_UE_DIR=${1:-"${CURRENT_DIR}"}
RCLUE_DIR="${TB3_UE_DIR}/Plugins/rclUE"

source ${RCLUE_DIR}/Scripts/setup_ros2libs.sh ${RCLUE_DIR}

#change default level
default_level=${LEVEL_NAME:-"Turtlebot3_benchmark"}
sed -e 's/${LEVEL_NAME}/'${default_level}'/g' Config/DefaultEngineBase.ini > Config/DefaultEngine.ini

UE_EDITOR=${2:-"$(dirname ${TB3_UE_DIR})/UnrealEngine/Engine/Binaries/Linux/UE4Editor"}
(exec "$UE_EDITOR" "${TB3_UE_DIR}/turtlebot3.uproject")
