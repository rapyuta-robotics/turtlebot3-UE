# Turtlebot UE

UE Project which includes examples to use rclUE.
## Branches
- `devel`: Main branch. Compatible with Unreal Engine 5.1
- `UE4`: Compatible with Unreal Engine 4.27

## Maps
- `ROS2Examples`: BP and C++ ROS2 example nodes of publisher/subscriber, service server/client and action server/client.
- `Turtlebot3 Benchmark`: BP and C++ ROS2 turtlebot3 navigation. Burger is implemented in C++ and Waffle is implemented in BP
- `Simple`: Simple robot arm example which can be controlled from [sensor_msgs/JointStates](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html)
- `Entry`: Get map name from command line and transition to that maps. Mainly used to packaged project to change initial map.
- `LargeGround`: Large enough map for simulating many robots. Mainly used to test [distributed multi-robot simulation](https://rapyutasimulationplugins.readthedocs.io/en/devel/distributed_simulation.html).
- `Turtlebot3AutoTest`: Maps for Automated test.

## Setup and run

1.  Download UE5.1 for Linux by following [Unreal Engine for Linux](https://www.unrealengine.com/en-US/linux)
2.  Clone this repo : `git clone --recurse-submodules git@github.com:rapyuta-robotics/turtlebot3-UE.git`
3.  Retrieve the large files : `git-lfs pull && git submodule foreach git-lfs pull`
4.  Build and run
    ```
    cd turtlebot3-UE
    export UE5_DIR=<path to UE5>
    ./update_project_files.sh
    ./make_editor.sh
    ./run_editor.sh $(pwd) <ue_exe>
    ```
## Install pre-commit
Please install pre-commit before commiting your changes.
Follow this instruction https://pre-commit.com/

then run

```bash
./setup_pre_commit.sh
```

this will setup pre-commit to all submodules as well.

## Turtlebot3 navigation

### Installation

1. [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
    * you can use ROS2 humble as well by checkout `Plugins/rclUE` to `UE5_devel_humble` branch.
2. [Install Nav2](https://navigation.ros.org/getting_started/index.html)

### Run

1. Play turtlebot3-UE
2. `ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False map:=<path to turtlebot3-UE>/Content/Turtlebot3_benchmark.yaml `

### Tests
!NOTE: The test script is setup to run with fastdds, which requires UE to start before ROS is enabled, thus `/opt/ros/<ros_distro>/setup.bash`, which is already run in the script, needs to be not added to `~/.bashrc`
```sh
./ExternalTest/run_local_sim_tb3_tests.sh <ue_exe> <ue_map> <tb3_model> <tb3_name> <tb3_init_pos> <tb3_init_rot>
```

with:

- `<ue_exe>`: path to the UE editor executor, eg: `~/UE/UnrealEngine/Engine/Binaries/Linux/UE4Editor`
- `<ue_map>`: ue map name, eg: `Turtlebot3AutoTest`
- `<tb3_model>`: `burger` or `waffle`
- `<tb3_name>` as the robot given names, eg: `burger0`
- `<tb3_init_pos>` as the robot initial position (x,y,z), eg: `0.0,0.0,0.1`
- `<tb3_init_rot>` as the robot initial rotation (r,p,y), eg: `0.0,0.0,0.0`
