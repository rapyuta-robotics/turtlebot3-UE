#!/bin/bash
Help()
{
    # Display Help
    echo "The script must be run from the turtlebot3-UE project dir."
    echo
    echo "Syntax: ./ExternalTest/$(basename $0) [-h] <tb3_model> <tb3_num>"
    echo "options:"
    echo "-h Print this Help."
    echo "arguments:"
    echo "tb3_model: tb3 model to be tested (burger or waffle), which must also have been defined as a key name of [SpawnableEntities] in the provided ue map"
    echo "tb3_num: num of tb3 robots"
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
    git clone https://github.com/rapyuta-robotics/UE_msgs.git "${RRSIM_TESTS_ROS_WS}/src/ue_msgs"
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
if [ ! -e ${TB3_TESTS_ROS_WS} ]; then
    mkdir -p ${TB3_TESTS_ROS_WS}/src
    cd ${TB3_TESTS_ROS_WS}/src
    ln -s ${TB3_TESTS_PKG_DIR} ${TB3_TESTS_PKG_NAME}
fi
cd ${TB3_TESTS_ROS_WS}
colcon build --symlink-install
source install/setup.bash

# Robot model: burger/waffle
ROBOT_MODEL=${1:-"turtlebot3_burger"}
ROBOTS_NUM=${2:-4}
if [[ ${ROBOT_MODEL} == *"burger"* ]]; then
    export TURTLEBOT3_MODEL=burger #
elif [[ ${ROBOT_MODEL} == *"waffle"* ]]; then
    export TURTLEBOT3_MODEL=waffle #
fi

printenv | grep TURTLEBOT3_MODEL

# (1)
echo "(1) Test whether sim state' service servers are ready"
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_sim_state.py timeout:='200'

# (2)
echo "(2) Test whether clock message is being published"
launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_clock_published.py

# (3)
echo "(3) Test multi-tb3 spawning with valid different namespaces"
for i in `seq 1 ${ROBOTS_NUM}`
do
    x=$(( ($i/${ROBOTS_NUM})*2))
    y=$(( ($i%${ROBOTS_NUM})*2 - 4))
    ROBOT_NAME="${ROBOT_MODEL}$i"

    launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_robot_spawn.py robot_model:=${ROBOT_MODEL} \
                                                               robot_name:=${ROBOT_NAME} \
                                                               robot_namespace:=${ROBOT_NAME} \
                                                               robot_pos:="$x, $y, 0.0" \
                                                               robot_rot:="0.0, 0.0, 0.0"
    
    # Test Laser being scanned
    if [ $i == 1 ]
    then
        echo "${ROBOT_NAME} - Test Laser being scanned"
        launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_laser_scan_published.py scan_topics:="/${ROBOT_NAME}/scan"
    fi

    # (3.1) Test odom being published
    echo "${ROBOT_NAME} - Test odom being published"
    launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_robot_odom_published.py robot_namespace:="${ROBOT_NAME}" is_tf_published:='True' is_tf_static:='False'

    # (3.2) Test cmd_vel being subscribed
    echo "${ROBOT_NAME} - Test cmd_vel being subscribed"
    launch_test ${RRSIM_TESTS_SCRIPTS_DIR}/test_robot_cmd_vel_subscribed.py robot_namespace:="${ROBOT_NAME}" robot_name:="${ROBOT_NAME}" \
                                                                            twist_linear:='0.2, 0.0, 0.0' \
                                                                            twist_angular:='0.0, 0.0, 1.8'
done

unset TURTLEBOT3_MODEL
