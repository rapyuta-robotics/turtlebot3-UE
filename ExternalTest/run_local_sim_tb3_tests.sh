#!/bin/bash

Help()
{
    # Display Help
    echo "The script must be run from the turtlebot3-UE project dir."
    echo
    echo "Syntax: ./ExternalTest/$(basename $0) [-h] <ue_exe> <ue_map> <tb3_model> <tb3_name> <tb3_init_pos> <tb3_init_rot>"
    echo "options:"
    echo "-h Print this Help."
    echo "arguments:"
    echo "ue_exe: Path to the ue executor. Eg: ~/UNREAL/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"
    echo "ue_map: ue map name. Eg: Turtlebot3AutoTest"
    echo "tb3_model: tb3 model to be tested (turtlebot3_burger or turtlebot3_waffle), of which its description folder (urdf/sdf + CAD) must be present under ExterntalData/RobotModels"
    echo "tb3_name: tb3 robot unique name"
    echo "tb3_init_pos: tb3's initial position (x,y,z), eg: 0.0,0.0,0.1"
    echo "tb3_init_rot: tb3's initial rotation (r,p,y), eg: 0.0,0.0,0.0"
    echo
}

# https://www.redhat.com/sysadmin/arguments-options-bash-scripts
while getopts ":h" option; do
    case $option in
        h) # display Help
            Help
            exit;;
        \?) # Invalid option
            echo "Error: Invalid option"
            Help
            exit;;
    esac
done

# Set exit-on-error mode
set -e

#CURRENT_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
#echo ${CURRENT_SCRIPT_DIR}

# This script is expected to be run from the project dir
TB3_UE_DIR="$(pwd -P)"
TB3_UE_DIR_NAME=${TB3_UE_DIR##*/}
if [[ "turtlebot3-UE" != "${TB3_UE_DIR_NAME}" ]]; then
    printf "TB3_UE_DIR_NAME: ${TB3_UE_DIR_NAME}\n"
    printf "${BASH_SOURCE[0]} must be run from turtlebot3-UE dir\n"
    Help
    #exit 1
fi

## GENERATE [Config/DefaultEngine.ini]
DEFAULT_LEVEL=${LEVEL_NAME:-"Turtlebot3_benchmark"}
sed -e 's/${LEVEL_NAME}/'${DEFAULT_LEVEL}'/g' ${TB3_UE_DIR}/Config/DefaultEngineBase.ini > ${TB3_UE_DIR}/Config/DefaultEngine.ini
## START turtlebot3-UE --
UE_EXE=$1
UE_MAP=${2:-"/Game/RapyutaRoom/Scenes/RapyutaRoom"}
#UE_MAP=${2:-"/RapyutaSimRobotImporter/Maps/RapyutaSingleSkeletalRobotDemo"}

# Set the domain ID prior to launching UE so that rclUE picks it up
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=${TB3_UE_DIR}/fastdds_config.xml

(exec "$UE_EXE" "${TB3_UE_DIR}/turtlebot3.uproject" "${UE_MAP}" "-game") &
RRSIM_PID="$(echo $!)"
echo "RRSIM PID: $RRSIM_PID"

# Give time for UE to finish init ROS & its plugins before running the script below, which must be afterwards
# ue5 takes much longer time than ue4 to init
sleep 120

## SETUP ROS TEST ENV --
# Note: This should be after UE4 has been brought up or it will break rclUE
source ${TB3_UE_DIR}/ExternalTest/setup_ros_test_env.sh

## START TB3 TESTS --
#
ROBOT_MODEL=${3:-"turtlebot3_burger"}
ROBOT_NAME=${4:-"burger0"}
ROBOT_INITIAL_POS=${5:-"-2.0,-1.0,2.5"} # z should be >= 0.1 is to avoid collision with the floor
ROBOT_INITIAL_ROT=${6:-"0.0,0.0,0.0"}
source ${TB3_UE_DIR}/ExternalTest/run_tb3_tests.sh ${ROBOT_MODEL} ${ROBOT_NAME} ${ROBOT_INITIAL_POS} ${ROBOT_INITIAL_ROT}

# Auto shutdown Sim
kill ${RRSIM_PID}
