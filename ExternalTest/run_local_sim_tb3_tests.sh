#!/bin/bash

set -e

Help()
{
    # Display Help
    echo "The script must be run from the turtlebot3-UE project dir."
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

## SETUP ROS TEST ENV --
cd ${TB3_UE_DIR}
source ${TB3_UE_DIR}/ExternalTest/setup_ros_test_env.sh

## START RRSIM --

# Generate [Config/DefaultEngine.ini]
DEFAULT_LEVEL=${LEVEL_NAME:-"Turtlebot3_benchmark"}
sed -e 's/${LEVEL_NAME}/'${DEFAULT_LEVEL}'/g' Config/DefaultEngineBase.ini > Config/DefaultEngine.ini

UE_EXE=$1
UE_MAP=${2:-"/Game/Maps/Turtlebot3AutoTest"}
$UE_EXE ${TB3_UE_DIR}/turtlebot3.uproject ${UE_MAP} -game &
RRSIM_PID="$(echo $!)"
echo "RRSIM PID: $RRSIM_PID"

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
