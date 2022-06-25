# Changelog for turtlebot3-UE repository

## 0.0.4 ##
* RapyutaRobotImporter:
- Skeletal robot add primitive mesh support, fix bone poses + physics constraints for simple_arm, tb3
- VHACD version 4
- DefaultEngineBase.ini Set global override game mode as RRRobotGameMode, game instance as RRGameInstance
- Add DefaultRapyutaSimSettings.ini Config `TEST_SKELETAL_ROBOT_MODEL_NAME` as one of {`simple_arm`, `turtlebot3_burger`, `turtlebot3_waffle`}

## 0.0.3 ##
* RapyutaSimulationPlugins: assets moved to lfs, ARRBaseRobot, ARRRobotBaseVehicle, URRRobotROS2Interface, URRTurtlebotROS2Interface
## 0.0.2 ##
* Update setup_ros_test_env.sh Pull ue_msgs from devel
* rclUE: ensure instead of check to avoid crash with duplicate subscriber (#48)
* RapyutaSimulationPlugins: add default subscribers + Joint Controllers #59, 60, 61

## 0.0.1 ##
* Add CHANGELOG.md
* rclUE's ROS2 fixed libs without using LD_LIBRARY_PATH
