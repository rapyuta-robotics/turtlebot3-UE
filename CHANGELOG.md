# Changelog for turtlebot3-UE repository

## 0.0.6 ##
* RapyutaRobotImporter:
- Skeletal robot add primitive mesh support, fix bone poses + physics constraints for simple_arm, tb3
- VHACD version 4
- DefaultEngineBase.ini Set global override game mode as RRRobotGameMode, game instance as RRGameInstance
- Add DefaultRapyutaSimSettings.ini Config `TEST_SKELETAL_ROBOT_MODEL_NAMES` as {`simple_arm`, `turtlebot3_burger`, `turtlebot3_waffle`} & `TEST_SKELETAL_ROBOT_NUM`
## 0.0.5 ##
* Enable cpp17 + ExternalTest fastdds
## 0.0.4 ##
* Update following UE-ROS adapter autogen #39
## 0.0.3 ##
* RapyutaSimulationPlugins: assets moved to lfs, ARRBaseRobot, ARRRobotBaseVehicle, URRRobotROS2Interface, URRTurtlebotROS2Interface
## 0.0.2 ##
* Update setup_ros_test_env.sh Pull ue_msgs from devel
* rclUE: ensure instead of check to avoid crash with duplicate subscriber (#48)
* RapyutaSimulationPlugins: add default subscribers + Joint Controllers #59, 60, 61

## 0.0.1 ##
* Add CHANGELOG.md
* rclUE's ROS2 fixed libs without using LD_LIBRARY_PATH
