#!/bin/bash

set -e

Help()
{
    # Display Help
    echo "The script must be run from the turtlebot3-UE project dir."
    echo
    echo "Syntax: ./ExternalTest/$(basename $0) [-h] <ue_exe> <ue_map> <tb3_model> <tb3_num>"
    echo "options:"
    echo "-h Print this Help."
    echo "arguments:"
    echo "ue_exe: Path to the ue executor. Eg: ~/UNREAL/UnrealEngine/Engine/Binaries/Linux/UE4Editor"
    echo "ue_map: ue map name. Eg: Turtlebot3AutoTest"
    echo "tb3_model: tb3 model to be tested (burger or waffle), which must also have been defined as a key name of [SpawnableEntities] in the provided ue map"
    echo "tb3_num: num of tb3 robots to be spawned"
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
UE_MAP=${2:-"/RapyutaSimRobotImporter/Maps/RapyutaSkeletalRobotLargeGroundDemo"}

# Set the domain ID prior to launching UE so that rclUE picks it up
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=${TB3_UE_DIR}/fastdds_config.xml

(exec "$UE_EXE" "${TB3_UE_DIR}/turtlebot3.uproject" "${UE_MAP}" "-game") &
RRSIM_PID="$(echo $!)"
echo "RRSIM PID: $RRSIM_PID"

# Give time for UE to come up and initialize its plugins or weird stuff happens because
# the below script is executed concurrently
sleep 5

## SETUP ROS TEST ENV --
# Note: This should be after UE4 has been brought up or it will break rclUE
source ${TB3_UE_DIR}/ExternalTest/setup_ros_test_env.sh

## START TB3 TESTS --
#
ROBOT_MODEL=${3:-"turtlebot3_burger"}
ROBOTS_NUM=${4:-20}
cd ${TB3_UE_DIR}
source ${TB3_UE_DIR}/ExternalTest/run_multi_tb3_tests.sh ${ROBOT_MODEL} ${ROBOTS_NUM}

# Auto shutdown Sim
#kill ${RRSIM_PID}
