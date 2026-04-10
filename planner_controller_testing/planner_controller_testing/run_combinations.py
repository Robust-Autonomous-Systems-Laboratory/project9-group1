#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
import yaml
import os

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

    rclpy.init()
    navigator = BasicNavigator()

    amcl_sub = navigator.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            amcl_cb,
            10)


    # load waypoint file directory from setup file
    pkg_share = get_package_share_directory('planner_controller_testing')
    filepath = os.path.join(pkg_share, 'external', 'waypoints.yaml')
 
    # load waypoints from project9-group1/config/waypoints.yaml (path given in setup.py)
    with open(filepath, 'r') as file:
        data = yaml.safe_load(file)
        points = data['waypoints']  # need to check format of loaded yaml file!
    
    waypoints = []

    # process yaml to list of PoseStamped() msgs
    for pt in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = points[pt]['pose'][0]
        pose.pose.position.y = points[pt]['pose'][1]
        pose.pose.orientation.z = points[pt]['orientation'][0]
        pose.pose.orientation.w = points[pt]['orientation'][3]
        waypoints.append(pose)

    #print(waypoints)
    
    # planner and controller combinations
    planner_combos = ['Dijkstra', 'AStar', 'AStar']
    controller_combos = ['RPP', 'RPP', 'DWB']
    num_combinations = len(planner_combos)
    
    # Wait for the navigation stack to be ready
    navigator.get_logger().info("Waiting for Nav2 stack...")
    navigator.waitUntilNav2Active()
    
    
    # cycle through all combinations
    for i in range(num_combinations):

        # navigator.goToPose does not support planner and controller parameters
        # Instead, getPath()/getPathThroughPoses() must be called first, followed by followPath(), as both support changing algorithms
        # See API here: https://docs.ros.org/en/iron/p/nav2_simple_commander/
        path = navigator.getPathThroughPoses(waypoints[0], waypoints[1:len(waypoints)-1], planner_id=planner_combos[i])
        navigator.followPath(path, controller_id=controller_combos[i])
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            navigator.get_logger().info("Navigating to waypoint {feedback.current_waypoint}")
            
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'Goal succeeded using {planner_combos[i]} planner and {controller_combos[i]} controller')
            # Calculate drift
            x, y = drift(lastPose, waypoints[4])
            navigator.get_logger().info(f"Dift accumulated x: {x}, y: {y}")
        elif result == TaskResult.CANCELED:
            navigator.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = navigator.getTaskError()
            navigator.get_logger().error('Goal failed!{error_code}:{error_msg}')
        else:
            navigator.get_logger().error('Goal has an invalid return status!')
        

    navigator.lifecycleShutdown()