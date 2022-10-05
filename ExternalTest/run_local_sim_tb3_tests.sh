#!/bin/bash

Help()
{
    # Display Help
    echo "The script must be run from the io_amr_ue project dir."
    echo
    echo "Syntax: ./ExternalTest/$(basename $0) [-h] <ue_exe> <ue_map> <tb3_model> <tb3_name> <tb3_init_pos> <tb3_init_rot>"
    echo "options:"
    echo "-h Print this Help."
    echo "arguments:"
    echo "ue_exe: Path to the ue executor. Eg: ~/UNREAL/UnrealEngine/Engine/Binaries/Linux/UE4Editor"
    echo "ue_map: ue map name. Eg: Turtlebot3AutoTest"
    echo "tb3_model: tb3 model to be tested (burger or waffle), which must also have been defined as a key name of [SpawnableEntities] in the provided ue map"
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
TB3_UE_DIR="$(pwd)"
TB3_UE_DIR_NAME=${TB3_UE_DIR##*/}
if [[ "turtlebot3-UE" != "${TB3_UE_DIR_NAME}" ]]; then
    printf "TB3_UE_DIR_NAME: ${TB3_UE_DIR_NAME}\n"
    printf "${BASH_SOURCE[0]} must be run from turtlebot3-UE dir\n"
    Help
    exit 1
fi

## START RRSIM --
UE_EXE=$1
UE_MAP=${2:-"Turtlebot3AutoTest"}

# Set the domain ID prior to launching UE so that rclUE picks it up
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=${TB3_UE_DIR}/fastdds_config.xml

# Change default level, generating DefaultEngine.ini
sed -e 's/${LEVEL_NAME}/'${UE_MAP}'/g' ${TB3_UE_DIR}/Config/DefaultEngineBase.ini > ${TB3_UE_DIR}/Config/DefaultEngine.ini

# Run turtlebot3-UE
$UE_EXE ${TB3_UE_DIR}/turtlebot3.uproject /Game/Maps/${UE_MAP} -game &
RRSIM_PID="$(echo $!)"
echo "RRSIM PID: $RRSIM_PID"

# Wait for UE to initialize its plugins, since the below script runs concurrently
sleep 5

## SETUP ROS TEST ENV --
cd ${TB3_UE_DIR}
source ${TB3_UE_DIR}/ExternalTest/setup_ros_test_env.sh

## START TB3 TESTS --
#
ROBOT_MODEL=${3:-"burger"}
ROBOT_NAME=${4:-"burger0"}
ROBOT_INITIAL_POS=${5:-"0.0,0.0,0.1"} # z should be >= 0.1 is to avoid collision with the floor
ROBOT_INITIAL_ROT=${6:-"0.0,0.0,0.0"}
cd ${TB3_UE_DIR}
source ${TB3_UE_DIR}/ExternalTest/run_tb3_tests.sh ${ROBOT_MODEL} ${ROBOT_NAME} ${ROBOT_INITIAL_POS} ${ROBOT_INITIAL_ROT}

# Auto shutdown Sim
kill ${RRSIM_PID}
