#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import argparse
import yaml

lastPose = None

def amcl_cb(msg):
    lastPose = msg

def drift(amcl : PoseWithCovarianceStamped, waypoint : PoseStamped):
    if lastPose is None:
        return -1.0, -1.0
    x_drift = abs(amcl.pose.pose.position.x - waypoint.pose.position.x)
    y_drift = abs(amcl.pose.pose.position.y - waypoint.pose.position.y)
    return x_drift, y_drift

def main()->None:
    # Get args
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", default=3, type=int)
    args, other = parser.parse_known_args()
    cycles = args.cycles

    rclpy.init(args=other)
    navigator = BasicNavigator()

    # amcl_sub = navigator.create_subscription(
    #         PoseWithCovarianceStamped,
    #         '/amcl_pose',
    #         amcl_cb,
    #         10)

 
    with open('./config/waypoints.yaml', 'r') as file:
        data = yaml.safe_load(file)
        points = data['waypoints']  # need to check format of loaded yaml file!
        print(points)

    waypoints = []

    for pt in points:

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt['pose'][0]
        pose.pose.position.y = pt['pose'][1]
        pose.pose.orientation.z = pt['orientation'][0]
        pose.pose.orientation.w = pt['orientation'][3]
        waypoints.append(pose)

    print("\n")
    print(waypoints)

    
    # # Wait for the navigation stack to be ready
    # navigator.get_logger().info("Waiting for Nav2 stack...")
    # navigator.waitUntilNav2Active()

 

    # # Start patrol loop
    # # task=follow_waypoints_task
    # for i in range(cycles):
    #     task = navigator.followWaypoints(waypoints)
    #     while not navigator.isTaskComplete():
    #         feedback = navigator.getFeedback()
    #         navigator.get_logger().info(f"Navigating to waypoint {feedback.current_waypoint}")

    #     result = navigator.getResult();
    #     if result == TaskResult.SUCCEEDED:
    #         navigator.get_logger().info('Goal succeeded!')
    #         # Calculate drift
    #         x, y = drift(lastPose, waypoints[4])
    #         navigator.get_logger().info(f"Dift accumulated x: {x}, y: {y}")
    #     elif result == TaskResult.CANCELED:
    #         navigator.get_logger().info('Goal was canceled!')
    #     elif result == TaskResult.FAILED:
    #         (error_code, error_msg) = navigator.getTaskError()
    #         navigator.get_logger().error('Goal failed!{error_code}:{error_msg}')
    #     else:
    #         navigator.get_logger().error('Goal has an invalid return status!')
        

    # navigator.lifecycleShutdown()