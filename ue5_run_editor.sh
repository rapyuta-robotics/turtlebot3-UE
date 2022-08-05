#!/bin/bash
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#source /home/cconti/Rapyuta/UnrealEngine/Engine/Build/BatchFiles/Linux/SetupEnvironment.sh -mono /home/cconti/Rapyuta/UnrealEngine/Engine/Build/BatchFiles/Linux;xbuild /property:Configuration=Development /verbosity:quiet /nologo /p:NoWarn=1591 /home/cconti/Rapyuta/UnrealEngine/Engine/Source/Programs/UnrealBuildTool/UnrealBuildTool.csproj
#mono /home/cconti/Rapyuta/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool.exe -ModuleWithSuffix=turtlebot3,2890 turtlebot3Editor Linux Development -Project="/home/cconti/Rapyuta/turtlebot3/turtlebot3.uproject" "/home/cconti/Rapyuta/turtlebot3/turtlebot3.uproject"  -IgnoreJunk -progress

CURRENT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
TB3_UE_DIR=${1:-"${CURRENT_DIR}"}

#change default level, generating DefautlEngine.ini
DEFAULT_LEVEL=${LEVEL_NAME:-"Turtlebot3_benchmark"}
sed -e 's/${LEVEL_NAME}/'${DEFAULT_LEVEL}'/g' Config/DefaultEngineBase.ini > Config/DefaultEngine.ini

UE_EDITOR=${2:-"$(dirname ${TB3_UE_DIR})/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"}
(exec "$UE_EDITOR" "${TB3_UE_DIR}/turtlebot3.uproject")
