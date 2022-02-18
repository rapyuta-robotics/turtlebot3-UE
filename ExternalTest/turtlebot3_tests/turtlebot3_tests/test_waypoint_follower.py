#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import os
import time
import unittest
from ament_index_python import get_package_share_directory

import launch
import launch.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import launch_testing.util
import pytest

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes

"""
Ref: https://github.com/ros-planning/navigation2/blob/main/nav2_system_tests/src/waypoint_follower/tester.py
"""
class WaypointFollower(Node):
    def __init__(self):
        super().__init__(node_name='nav2_waypoint_tester', namespace='')
        self.waypoints = None
        self.action_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)
        self.initial_pose_received = False
        self.goal_handle = None

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)

        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose', self.poseCallback, pose_qos)

    def setInitialPose(self, pose):
        from tf_transformations import quaternion_from_euler
        assert(len(pose) >= 6)
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.pose.pose.position.z = pose[2]

        q = quaternion_from_euler(pose[3], pose[4], pose[5])
        self.init_pose.pose.pose.orientation.x = q[0]
        self.init_pose.pose.pose.orientation.y = q[1]
        self.init_pose.pose.pose.orientation.z = q[2]
        self.init_pose.pose.pose.orientation.w = q[3]

        self.init_pose.header.frame_id = 'map'
        self.publishInitialPose()
        time.sleep(5)

    def poseCallback(self, msg):
        self.info_msg('Received amcl_pose')
        self.initial_pose_received = True

    def setWaypoints(self, in_waypoints):
        self.waypoints = []

        for wp in in_waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self, block):
        if (not self.waypoints) or (len(self.waypoints) == 0):
            rclpy.error_msg('Did not set valid waypoints before running test!')
            return False

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'FollowWaypoints' action server not available, waiting...")

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        if not block:
            return True

        get_result_future = self.goal_handle.get_result_async()

        self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.error_msg('Goal failed with status code: {0}'.format(status))
            return False

        if len(result.missed_waypoints) > 0:
            self.warn_msg('Goal failed to process all waypoints,'
                          ' missed {0} wps.'.format(len(result.missed_waypoints)))
        self.info_msg('Goal succeeded!')
        return True

    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def shutdown_nav_lifecycle(self):
        self.info_msg('Shutting down')
        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        transition_service = 'lifecycle_manager_localization/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

    def cancel_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)

def follow_waypoints(in_waypoints, in_initial_pose):
    # wait a few seconds to make sure entire stacks are up
    time.sleep(10)

    test = WaypointFollower()
    test.setWaypoints(in_waypoints)

    retry_count = 0
    retries = 2
    while not test.initial_pose_received and retry_count <= retries:
        retry_count += 1
        test.info_msg(f'Setting initial pose {in_initial_pose}')
        test.setInitialPose(in_initial_pose)
        test.info_msg('Waiting for amcl_pose to be received')
        rclpy.spin_once(test, timeout_sec=1.0)  # wait for poseCallback

    result = test.run(block=True)
    if not result:
        test.error_msg('Following waypoints FAILED')
    else:
        test.info_msg('Following waypoints PASSED')
    test.shutdown_nav_lifecycle()
    # Let the node by garbage collected due to destroying it here may compromise the nav stack shutdown
    return result

"""
Test basic navigation following waypoints
"""

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # [tb3_model] arg
    tb3_model = launch.substitutions.LaunchConfiguration('tb3_model', default='burger')

    # [waypoints] arg
    waypoints = launch.substitutions.LaunchConfiguration('waypoints', default='-0.52, -0.78, 0.7, 0.5, 2.0, -1.5, 1.7, 1.7')

    # [initial_pose] arg
    initial_pose = launch.substitutions.LaunchConfiguration('initial_pose', default='-1.0, 0.0, 0.0, 0.0, 0.0, 0.0')

    # Bringup the turtlebot3
    tb3_robot_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch/robot.launch.py')),
        launch_arguments={'use_sim_time': 'False'}.items()
    )

    # Start tb3 nav2
    tb3_nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/tb3_simulation_launch.py')),
        launch_arguments={'use_simulator': 'False', 'headless': 'True', 'map':'maps/Turtlebot3_benchmark.yaml'}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'tb3_model',
            default_value=tb3_model,
            description='turtlebot3 model (burger or waffle)'),
        launch.actions.DeclareLaunchArgument(
            'waypoints',
            default_value=waypoints,
            description="Waypoints. Eg: '-0.52, -0.78, 0.7, 0.5, 2.0, -1.5, 1.7, 1.7'"),
        launch.actions.DeclareLaunchArgument(
            'initial_pose',
            default_value=initial_pose,
            description="Initial tb3 pose, 'pos_x, pos_y, pos_z, roll, pitch, yaw'"),
        launch.actions.SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),
        tb3_robot_launch,
        tb3_nav2_launch,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ])

class TestWaypointFollower(unittest.TestCase):
    def test_waypoint_follower(self, proc_output, test_args):
        rclpy.init()
        waypoints = []
        waypoints_arg = [float(x.strip()) for x in test_args['waypoints'].split(',')]
        waypoints_num = len(waypoints_arg)
        assert(waypoints_num % 2 == 0)
        for i in range(int(waypoints_num/2)):
            waypoints.append([waypoints_arg[2*i], waypoints_arg[2*i+1]])

        initial_pose = [float(x.strip()) for x in test_args['initial_pose'].split(',')]
        assert follow_waypoints(waypoints, initial_pose), 'Waypoint failed being followed!'
        rclpy.shutdown()
