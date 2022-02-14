# turtlebot3 tests
This package contains test cases using the ``launch`` and ``launch_testing`` packages.

## Examples

### `test_waypoint_follower.py`

Usage:

```sh
mkdir maps
ln ../../Content/Turtlebot3_benchmark.pgm maps/Turtlebot3_benchmark.pgm
ln ../../Content/Turtlebot3_benchmark.yaml maps/Turtlebot3_benchmark.yaml
launch_test src/test_waypoint_follower.py tb3_model:=burger
```

This test randomly generates a goal pose that the turtlebot3 is expected to navigate through pre-defined waypoints
