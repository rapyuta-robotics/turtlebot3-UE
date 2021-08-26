#!/bin/sh

#source /home/cconti/Rapyuta/UnrealEngine/Engine/Build/BatchFiles/Linux/SetupEnvironment.sh -mono /home/cconti/Rapyuta/UnrealEngine/Engine/Build/BatchFiles/Linux;xbuild /property:Configuration=Development /verbosity:quiet /nologo /p:NoWarn=1591 /home/cconti/Rapyuta/UnrealEngine/Engine/Source/Programs/UnrealBuildTool/UnrealBuildTool.csproj
#mono /home/cconti/Rapyuta/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool.exe -ModuleWithSuffix=turtlebot3,2890 turtlebot3Editor Linux Development -Project="/home/cconti/Rapyuta/turtlebot3/turtlebot3.uproject" "/home/cconti/Rapyuta/turtlebot3/turtlebot3.uproject"  -IgnoreJunk -progress

export LD_LIBRARY_PATH=\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/std_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/geometry_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/sensor_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/rosgraph_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/nav_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/tf2_msgs/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/builtin_interfaces/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/ue4_interfaces/lib":\
"${PWD}/Plugins/rclUE/Source/ThirdParty/ros2lib/ue_msgs/lib"

#export ROS_MASTER_URI=http://localhost:11311

unset ROS_DOMAIN_ID
EDITOR_COMMAND="../UnrealEngine/Engine/Binaries/Linux/UE4Editor"

#change default level
default_level=${LEVEL_NAME:-"Turtlebot3_benchmark"}
sed -e 's/${LEVEL_NAME}/'${default_level}'/g' Config/DefaultEngineBase.ini > Config/DefaultEngine.ini

(exec "$EDITOR_COMMAND" "${PWD}/turtlebot3.uproject")
