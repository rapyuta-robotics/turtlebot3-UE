# Turtlebot UE

UE Project which includes examples to use rclUE.

## Documentation
- [rclUE](): This repo enables communication between UE and ROS 2.
- [RapyutaSimulationPlugins](): This repo has classes/tools to create ROS 2 enables robots with rclUE.
## Branches
- `devel`: Main branch. Compatible with Unreal Engine 5.1 with Ubuntu 20.04
- `jammy`: Compatible with Unreal Engine 5.1 with Ubuntu 22.04
- `UE4`: Compatible with Unreal Engine 4.27

## Maps
- Base ROS2 examples
    - `ROS2TopicExamples`: BP and C++ ROS2 example nodes of publisher/subscriber.
    - `ROS2ServiceExamples`: BP and C++ ROS2 example nodes of service server/client.
    - `ROS2ActionExamples`: BP and C++ ROS2 example nodes of action server/client
- Robot Examples(explanation of [robots](https://rapyutasimulationplugins.readthedocs.io/en/devel/robots.html))
    - `Turtlebot3 Benchmark`: BP and C++ ROS2 turtlebot3 navigation. Burger is implemented in C++ and Waffle is implemented in BP.
    - `RobotArmExample`: Robot arm example which can be controlled from [sensor_msgs/JointStates](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html). This map has SimpleArim, KinematicUR10 and PhysicsUR10.
    - `PandaArmExample`: Panda Arm example which can be controlled from moveit2
    - `PandaArmPhysicsExample`: Physics Panda Arm example which can be controlled from moveit2
- Others
    - `Entry`: Get map name from command line and transition to that maps. Mainly used to packaged project to change initial map.
    - `LargeGround`: Large enough map for simulating many robots. Mainly used to test [distributed multi-robot simulation](https://rapyutasimulationplugins.readthedocs.io/en/devel/distributed_simulation.html).
    - `Turtlebot3AutoTest`: Maps for Automated test.

## Setup and run
* Please check [Getting Started](https://rapyutasimulationplugins.readthedocs.io/en/doc_update/getting_started.html) as well.

__(1) Download the latest UE5 for Linux__ by following [Unreal Engine for Linux](https://www.unrealengine.com/en-US/linux)
__(2) Setup `UE5_DIR` environment variable__ as the full path to the installed `UnrealEngine` folder.
__(3) Clone the repository with all required submodules__
```
git clone --recursive git@github.com:rapyuta-robotics/turtlebot3-UE.git
git-lfs pull && git submodule foreach git-lfs pull
```
__(4) Generate VSCode build configurations__
```
cd turtlebot3-UE
./update_project_files.sh
```

If the script fails because of obsolete temporary files, remove them:
```
./cleanup.sh
```
__(5) Build turtlebot3-UEEditor project__
```
./make_editor.sh
```
Building can also be done inside VSCode:
- Launch VSCode and open folder "turtlebot3-UE"
- Press `Ctrl+Shift+B` and choose "turtlebot3-UEEditor Linux Development Build"

# Run
__Running in UE Editor__
```
./run_editor.sh <discover_server> <ue_project_dir>
```
, where
* `<discovery_server>`: if `true`, will run FastDDS setup script, default: `true`. [ROS2 with Discovery Server](https://docs.ros.org/en/foxy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)
* `ue_project_dir`: full path to the ue-client project dir, default: current directory

__Running as Standalone game__
```
./run_uncooked_standalone.sh
```

__Required system env variables__
* `LEVEL_NAME`: The initially loaded level name, default: `Turtlebot3_benchmark`
* `FIXED_FRAME_RATE`: Fixed frame rate, default: `100.0`
* `TARGET_RTF`: Real-time factor, default: `1.0`

# Developer setup
1. Install and setup [git-lfs](https://git-lfs.github.com/) for all submodules.
2. Setup hook path to turtlebot3-UE directory
```
cd turtlebot3-UE
git config --local core.hooksPath .
```
3. Install [pre-commit](https://pre-commit.com)
then run

```bash
./setup_pre_commit.sh
```
to setup pre-commit to all submodules as well.

4. Refer to [UE Sim design guidelines](https://docs.google.com/document/d/1J4xI-68gF0aGdTA-NmieshEHdHWT-8dP-goKJlMIXwM) for further setup info

# Configuration
## User UE Configs
* Compose a custom [UE INI](https://docs.unrealengine.com/5.2/en-US/configuration-files-in-unreal-engine) file named `RapyutaSimSettings.ini` and put it inside `Config` folder (or Mount it there if running from turtlebot3-UE docker image).

* Detailed configs are described in `Config/DefaultRapyutaSimSettings.ini`

# Package turtlebot3-UE
```
./package_client.sh <output_archive_dir>
```
, where `<output_archive_dir>` is the full path to an output folder for the package.

Then run the packaged build by
```
./run_packaged.sh <turtlebot3-UE_dir> <output_package_dir>
```

## Turtlebot3 navigation

### Installation

1. [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
    * you can use ROS2 humble as well by checkout `Plugins/rclUE` to `UE5_devel_humble` branch.
2. [Install Nav2](https://navigation.ros.org/getting_started/index.html)

### Run

1. Play turtlebot3-UE
2. `cd turtlebot3-UE && source fastdds_setup.sh` #if you use ROS2 Discovery Server. You don't need this if you start editor with `./run_editor false`.
3. `ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False map:=<path to turtlebot3-UE>/Content/Turtlebot3_benchmark.yaml `

### Tests
!NOTE: The test script is setup to run with fastdds, which requires UE to start before ROS is enabled, thus `/opt/ros/<ros_distro>/setup.bash`, which is already run in the script, needs to be NOT added to `~/.bashrc`
1. Run turtlebot3-UE in PIE
2. In a separate terminal, run
```sh
./ExternalTest/run_local_sim_tb3_tests.sh <ue_exe> <ue_map> <frame_rate> <rtf> <tb3_model> <tb3_name> <tb3_init_pos> <tb3_init_rot>
```

with:

- `<ue_exe>`: path to the UE editor executor, default: `${UE5_DIR}/Engine/Binaries/Linux/UnrealEditor`
- `<ue_map>`: ue map name, default: `Turtlebot3AutoTest`
- `<frame_rate>`: fixed frame rate, default: `100.0`
- `<rtf>`: real-time factor, default: `1.0`
- `<tb3_model>`: tb3 model (`turtlebot3_burger`, `BP_TurtlebotBurger`, `turtlebot3_waffle`, `BP_TurtlebotWaffle`) to be tested, default: `turtlebot3_burger`
- `<tb3_name>` as the robot given names, default: `burger0`
- `<tb3_init_pos>` as the robot initial position (x,y,z), default: `0.0,0.0,0.1`
- `<tb3_init_rot>` as the robot initial rotation (r,p,y), default: `0.0,0.0,0.0`
