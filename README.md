# Turtlebot UE

UE4 Project which includes examples to use rclUE.

## Maps

- PubSub: BP and C++ ROS2 publisher subscriber example.
- Turtlebot3 Benchmark: BP and C++ ROS2 turtlebot3 navigation. Burger is implemented in C++ and Waffle is implemented in BP

## Setup and run

1.  Setup UE4 in Linux by following [Linux Quick Start](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/)
2.  clone this repo
3.  Build and run
    ```
    cd turtlebot3-UE
    ./update_project_files.sh
    ./make_editor.sh
    ./run_editor.sh $(pwd) <ue_exe>
    # Eg: ./run_editor.sh $(pwd) ~/UE/UnrealEngine/Engine/Binaries/Linux/UE4Editor
    ```

## Turtlebot3 navigation

### Installation

1. [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
2. [Install Nav2](https://navigation.ros.org/getting_started/index.html)

### Run

1. Play turtlebot3-UE
2. `ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False map:=<path to turtlebot3-UE>/Content/Turtlebot3_benchmark.yaml `

### Tests

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
