# Project 9: Path Planning and Following
Reid Beckes, Jackson Newell, Ian Mattson, and Anders Smitterberg


# Introduction + Setup

-

This project is completed and tested in ROS2 Jazzy Jalisco on an Ubuntu 24.04 Noble Numbat PC.

Each Turtlebot3 in EERC 722 is assigned a static IP on a lab managed wireless router. Our group used Turtlebot Anchovy, which is assigned local IP address 32.80.100.108 and `ROS_DOMAIN_ID=8`.

The testing environment is setup on a local PC by exporting the following parameter flags:
```bash
$ export ROS_DOMAIN_ID=8
$ export TURTLEBOT3_MODEL=burger
$ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

The Turtlebot3 modified Nav2 parameters are located here: [`/config/nav2_params.yaml`](./config/nav2_params.yaml). Further instructions on launching the Turtlebot3 Nav2 node are in the Usage Instructions section.

# Part 1 - Route Design

## Route Justification

-

## Route Map

-

## Start and Goal Poses

| | x | y | yaw |
| :--: | :--: | :--: | :--: |
| Start |  |  |  |
| Goal  |  |  |  |


# Part 2 - Planner and Controller Comparison

## 2a. Planner Comparison (Dijkstra vs. A\*)

-

## 2b. Controller Comparison (RPP vs. DWB)

-

## 2c. Run Results Summary

| Run | Planner | Controller | Observations |
|-----|---------|------------|--------------|
| 1 | Dijkstra | RPP |  |
| 2 | A\* | RPP |  |
| 3 | A\* | DWB |  |


# Part 3 - Analysis

## Planner Comparison Analysis

**Did Dijkstra and A\* produce identical or different paths?**

-

**Is the straight-line heuristic informative or misleading near the keepout zone boundary?**

-

## Controller Comparison Analysis

**Describe one moment where RPP and DWB produced observably different behavior.**

-

**Was `lookahead_dist: 0.4 m` a good choice for your route?**

-


# Usage Instructions

All commands are run from the workspace root (`proj9_ws/`). Source the workspace and set the robot model before running anything:

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=8
source install/setup.bash
```

### Terminal 1 — TurtleBot3 bringup (SSH into robot, run on the robot)
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### Terminal 2 — Costmap filter servers (workstation)
```bash
ros2 launch src/config/filters_launch.py
```
Wait until you see `Managed nodes are active` before proceeding.

### Terminal 3 — Nav2 with both planners and controllers (workstation)
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=False \
  map:=src/maps/map_eerc722.yaml \
  params_file:=src/config/nav2_params.yaml
```

Once Nav2 is up, use RViz2's **2D Pose Estimate** to initialize AMCL before sending any navigation goals.

### Terminal 4 — Run all planner × controller combinations
```bash
ros2 run scripts run_combinations.py
```

### Selecting a specific planner and controller combination at runtime

-


# AI Disclosure

**Anders Smitterberg**

-

**Reid Beckes**

-

**Jackson Newell**

-

**Ian Mattson**

-
