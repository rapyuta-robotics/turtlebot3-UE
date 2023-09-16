#!/bin/bash

PROJECT_DIR_NAME="turtlebot3-UE"
PROJECT_NAME=${PROJECT_DIR_NAME}
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
    exit 1
fi

## START RRSIM --
UE_EXE=${1:-"${UE5_DIR}/Engine/Binaries/Linux/UnrealEditor"}
RRSIM_TARGET_LEVEL=${2:-"/Game/Maps/Turtlebot3AutoTest"}
RRSIM_FRAME_RATE=${3:-"100.0"} #Hz
RRSIM_RTF=${4:-"1.0"} # Real-time factor

# Whether auto running Sim as standalone game or requiring prior PIE before tests
# NOTE: Sim local auto test has unstable ROS2 communication, thus defaulted as "0"
RRSIM_AUTO_TEST=${5:-"0"}

# Set the domain ID prior to launching UE so that rclUE picks it up
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=${TB3_UE_DIR}/fastdds_config.xml

# RUN RRSIM if Auto test
if [[ "${RRSIM_AUTO_TEST}" == "1" ]]; then
    # Generate DefaultEngine.ini, setting-up dynamic configs
    sed -e 's/${LEVEL_NAME}/'${RRSIM_TARGET_LEVEL}'/g' Config/DefaultEngineBase.ini > Config/DefaultEngine.ini
    sed -i -e 's/${FIXED_FRAME_RATE}/'${RRSIM_FRAME_RATE}'/g' Config/DefaultEngine.ini
    sed -i -e 's/${TARGET_RTF}/'${RRSIM_RTF}'/g' Config/DefaultEngine.ini

    # Run ${PROJECT_NAME} async
    (exec "$UE_EXE" "${TB3_UE_DIR}/${PROJECT_NAME}.uproject" "-game" "-targetmap=${RRSIM_TARGET_LEVEL}") &
    RRSIM_PID="$(echo $!)"
    # Note: This must be after UE5 has been fully brought up or it will break rclUE
    sleep 120
else
    RRSIM_PID="$(echo $(ps -a | grep -E "UnrealEditor|${PROJECT_NAME}"))"
    if [[ "${RRSIM_PID}" == "" ]]; then
        echo "Make sure RRSIM has been run as packaged build or PIE/Standalone (in a separate terminal window!) beforehand"
        exit 1
    fi
fi

## SETUP ROS TEST ENV --
# Note: This should be after UE has been brought up or it will break rclUE
# To avoid args reusing in [setup_ros_test_env.sh] without explicit args passed here
run_setup_ros_test_env() {
    source ${TB3_UE_DIR}/ExternalTest/setup_ros_test_env.sh
}
run_setup_ros_test_env

## START TB3 TESTS --
#
# Robot model: turtlebot3_burger/BP_TurtlebotBurger or turtlebot3_waffle/BP_TurtlebotWaffle
ROBOT_MODEL=${6:-"turtlebot3_burger"}
ROBOT_NAME=${7:-"burger0"}
ROBOT_INITIAL_POS=${8:-"0.0,0.0,0.1"} # z should be >= 0.1 is to avoid collision with the floor
ROBOT_INITIAL_ROT=${9:-"0.0,0.0,0.0"}
source ${TB3_UE_DIR}/ExternalTest/run_tb3_tests.sh ${ROBOT_MODEL} ${ROBOT_NAME} ${ROBOT_INITIAL_POS} ${ROBOT_INITIAL_ROT}

# Auto shutdown Sim
kill ${RRSIM_PID}
