#!/bin/bash

if [ -z "${UE5_DIR}" ]; then
	printf "Please set UE5_DIR to path of UE5 UnrealEngine's parent folder\n"
	exit 1
fi

PROJ_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
${UE5_DIR}/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool Development Linux -Project="${PROJ_DIR}/turtlebot3.uproject" -TargetType=Editor #-Progress -NoEngineChanges -NoHotReloadFromIDE
