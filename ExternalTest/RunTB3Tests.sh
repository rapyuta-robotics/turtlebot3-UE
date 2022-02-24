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

# This script is expected to be run from the project dir
TB3_UE_DIR="$(pwd)"

## START RRSIM --
#
UE_EXE=$1
UE_MAP=${2:-"Turtlebot3AutoTest"}
$UE_EXE ${TB3_UE_DIR}/turtlebot3.uproject /Game/Maps/${UE_MAP} -game &
RRSIM_PID="$(echo $!)"
echo "RRSIM PID: $RRSIM_PID"

## START TESTS --
#
# Build [rr_sim_tests] pkg
RRSIM_TESTS_ROS_WS="${TB3_UE_DIR}/Plugins/RapyutaSimulationPlugins/ExternalTest/RRSIM_TESTS"
RRSIM_TESTS_PKG_NAME='rr_sim_tests'
RRSIM_TESTS_PKG_DIR="${TB3_UE_DIR}/Plugins/RapyutaSimulationPlugins/ExternalTest/${RRSIM_TESTS_PKG_NAME}"
RRSIM_TESTS_SCRIPTS_DIR="${RRSIM_TESTS_PKG_DIR}/${RRSIM_TESTS_PKG_NAME}"

if [ ! -e ${RRSIM_TESTS_ROS_WS} ]; then
    mkdir -p ${RRSIM_TESTS_ROS_WS}/src
    ln -s ${RRSIM_TESTS_PKG_DIR} "${RRSIM_TESTS_ROS_WS}/src/${RRSIM_TESTS_PKG_NAME}"
fi
cd ${RRSIM_TESTS_ROS_WS}
colcon build --symlink-install
source install/setup.bash

# Build [turtlebot3_tests] pkg
TB3_TESTS_ROS_WS="${TB3_UE_DIR}/ExternalTest/TB3_TESTS"
TB3_TESTS_PKG_NAME='turtlebot3_tests'
TB3_TESTS_PKG_DIR="${TB3_UE_DIR}/ExternalTest/${TB3_TESTS_PKG_NAME}"
TB3_TESTS_SCRIPTS_DIR="${TB3_TESTS_PKG_DIR}/${TB3_TESTS_PKG_NAME}"

cd ${TB3_TESTS_PKG_DIR}
mkdir -p maps
if [ ! -f maps/Turtlebot3_benchmark.pgm ]; then
    ln ../../Content/Turtlebot3_benchmark.pgm maps/Turtlebot3_benchmark.pgm
fi
if [ ! -f maps/Turtlebot3_benchmark.yaml ]; then
    ln ../../Content/Turtlebot3_benchmark.yaml maps/Turtlebot3_benchmark.yaml
fi

if [ ! -e ${TB3_TESTS_ROS_WS} ]; then
    mkdir -p ${TB3_TESTS_ROS_WS}/src
    ln -s ${TB3_TESTS_PKG_DIR} "${TB3_TESTS_ROS_WS}/src/${TB3_TESTS_PKG_NAME}"
fi
cd ${TB3_TESTS_ROS_WS}
colcon build --symlink-install
source install/setup.bash

# Robot model: burger/waffle
ROBOT_MODEL=${3:-"burger"}
ROBOT_NAME=${4:-"burger0"}
ROBOT_INITIAL_POS=${5:-"0.0,0.0,0.1"} # z should be >= 0.1 is to avoid collision with the floor
ROBOT_INITIAL_ROT=${6:-"0.0,0.0,0.0"}
export TURTLEBOT3_MODEL=${ROBOT_MODEL}
printenv | grep TURTLEBOT3_MODEL

# Test whether sim state' service servers are ready
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_sim_state.py timeout:='200'

# Test whether clock message is being published
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_clock_published.py

# Test robot spawning  with empty robot namespace & its odom publication
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_robot_spawn.py robot_model:=${ROBOT_MODEL} \
                                                           robot_name:=${ROBOT_NAME} \
                                                           robot_pos:=${ROBOT_INITIAL_POS} \
                                                           robot_rot:=${ROBOT_INITIAL_ROT}

# Test Laser being scanned
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_laser_scan_published.py scan_topics:="/scan"

# Test odom being published
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_robot_odom_published.py is_tf_published:='True' is_tf_static:='False'

# Test robot auto-navigation
cd ${TB3_TESTS_PKG_DIR}
launch_test ${TB3_TESTS_SCRIPTS_DIR}/test_waypoint_follower.py waypoints:='-0.52, -0.78, 0.7, 0.5, 2.0, -1.5, 1.7, 1.7'\
                                                               initial_pose:="${ROBOT_INITIAL_POS}, ${ROBOT_INITIAL_ROT}"

# Test robot removal
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_robot_remove.py robot_name:=${ROBOT_NAME}

unset TURTLEBOT3_MODEL

# Auto shutdown Sim
kill ${RRSIM_PID}
