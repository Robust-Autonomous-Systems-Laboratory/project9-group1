#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def main()->None:

    rclpy.init()
    navigator = BasicNavigator()

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
    
    # Wait for the navigation stack to be ready
    navigator.get_logger().info("Waiting for Nav2 stack...")
    navigator.waitUntilNav2Active()

    # begin navigating path combinations!
    navigator.get_logger().info("Starting combo 1: Dijkstra and RPP")
    # navigator.goToPose does not support planner and controller parameters
    # Instead, getPath()/getPathThroughPoses() must be called first, followed by followPath(), as both support changing algorithms
    # See API here: https://docs.nav2.org/commander_api/index.html
   
    path = navigator.getPathThroughPoses(waypoints[0], waypoints, planner_id='Dijkstra')
    navigator.followPath(path, controller_id='RPP')
            
    while not navigator.isTaskComplete():
        pass
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Goal was canceled!')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('Goal failed!')
    else:
        navigator.get_logger().info('Goal has an invalid return status!')
        
    # trigger slight move so the next cycle can start without prematurely ending (not already in goal pose)
    navigator.get_logger().info("Path following complete, manually adjusting for next run")
    navigator.spin(spin_dist=3.14, time_allowance=15)
    while not navigator.isTaskComplete():
        pass
    
    navigator.driveOnHeading(dist=0.25, speed=0.2, time_allowance=25)
    while not navigator.isTaskComplete():
        pass
    
    
    # start combination 2, A* planner and RPP controller
    navigator.get_logger().info("Starting combo 2: A* and RPP")
    path = navigator.getPathThroughPoses(waypoints[0], waypoints, planner_id='AStar')
    navigator.followPath(path, controller_id='RPP')
            
    while not navigator.isTaskComplete():
        pass
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Goal was canceled!')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('Goal failed!')
    else:
        navigator.get_logger().info('Goal has an invalid return status!')
    
     # trigger slight move so the next cycle can start without prematurely ending (not already in goal pose)
    navigator.get_logger().info("Path following complete, manually adjusting for next run")
    navigator.spin(spin_dist=3.14, time_allowance=15)
    while not navigator.isTaskComplete():
        pass
    
    navigator.driveOnHeading(dist=0.25, speed=0.2, time_allowance=25)
    while not navigator.isTaskComplete():
        pass
    
    # start combination 3, A* planner and DWB controller
    navigator.get_logger().info("Starting combo 3: A* and DWB")
    path = navigator.getPathThroughPoses(waypoints[0], waypoints, planner_id='AStar')
    navigator.followPath(path, controller_id='DWB')
            
    while not navigator.isTaskComplete():
        pass
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Goal was canceled!')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('Goal failed!')
    else:
        navigator.get_logger().info('Goal has an invalid return status!')
   
    navigator.get_logger().info("All combinations ended, Node Terminating...")

    navigator.lifecycleShutdown()