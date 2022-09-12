#!/bin/bash
if [ -z "${UE5_DIR}" ]; then
	printf "Please set UE5_DIR to path of UE5 UnrealEngine's parent folder\n"
	exit 1
fi

TB3_UE_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v20_clang-13.0.1-centos7/x86_64-unknown-linux-gnu/lib64/
${UE5_DIR}/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool Development Linux -Project="${TB3_UE_DIR}/turtlebot3.uproject" -TargetType=Editor #-Progress -NoEngineChanges -NoHotReloadFromIDE
