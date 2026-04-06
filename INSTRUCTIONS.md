# Project 9: Path Planning and Following

## EE5531 Introduction to Robotics

---

## Introduction

In lecture you have seen how path planning algorithms find collision-free routes through configuration space and how local controllers drive the robot along those routes. In this project you take those algorithms off the whiteboard and onto the TurtleBot3 in EERC 722.

You will compare two global planners on a single carefully chosen route, then compare two local controllers driving that same planned path. The route is not arbitrary — it must force both the planner and the controller to make interesting decisions.

This project uses the EERC 722 map and Nav2 configuration from Project 8 as its starting point.

---

## Learning Objectives

1. Predict and explain the difference between Dijkstra and A\* on a route that exercises your Project 8 keepout zone
2. Observe and explain the behavioral difference between Regulated Pure Pursuit and DWB when following the same planned path
3. Connect the lookahead distance trade-off and the velocity space search concept from lecture to observable robot behavior on hardware

---

## Team Structure

This project is completed in team up to four or on your own. All team members must make meaningful commits to the repository before the submission deadline. Graders will check the git log — commits that only add image files or cosmetic README changes do not count as meaningful contributions.

---

## AI Use Policy

Any use of AI tools (ChatGPT, Claude, Copilot, etc.) **must be thoroughly documented** in your README. For each section where AI was used, note:

- Which tool was used
- What prompt or query was given
- How you verified or modified the output

Undisclosed AI use is an academic integrity violation.

---

## Background

### Planner vs. Controller

The Nav2 navigation pipeline has two distinct decision-making layers:

```
Goal Pose
   │
   ▼
[ Planner Server ]   global path (list of poses) — runs once per goal
   │
   ▼
[ Controller Server ] cmd_vel (linear.x, angular.z) — runs at ~10 Hz
   │
   ▼
[ Robot ]
```

The **planner** operates on the full global costmap and produces a collision-free path. It runs once when a goal is set, and again whenever replanning is triggered.

The **controller** receives that path and produces velocity commands at the robot's control rate. It has no knowledge of the full map — only the local costmap (~3 m radius) updated from live sensor data.

These two layers can be varied independently. The same global plan can be followed by different controllers with measurably different behavior.

### Global Planners

Nav2's `NavfnPlanner` implements both Dijkstra and A\* via a single parameter:

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  use_astar: false   # false = Dijkstra, true = A*
```

On typical indoor routes in free space, the two algorithms find paths of nearly identical length — A\* is faster to compute, but the paths are the same. The interesting question for this project is: *when do they diverge?* The answer is in routes where the costmap's cost gradient (from your Project 8 inflation radius and keepout zone) creates a situation where A\*'s heuristic pulls it toward a path that Dijkstra would also find, but through different intermediate nodes — occasionally producing a visible path difference near constrained regions.

### Local Controllers

Nav2 provides two controllers relevant to this project:

**Regulated Pure Pursuit (RPP)** computes the arc that connects the robot's current pose to a point exactly `lookahead_dist` meters ahead on the global path. The robot "chases" this lookahead point. The result is a purely geometric tracker — it does not reason about obstacles, only about path curvature and proximity regulation. It is predictable and easy to tune.

**DWB (Dynamic Window Approach with critics)** samples hundreds of short velocity trajectories, simulates them forward ~1 second, and scores each using a weighted sum of critics (path alignment, goal alignment, obstacle distance). It selects the trajectory with the highest score. Because it scores trajectories against the local costmap, DWB actively steers around obstacles rather than just slowing near them.

The behavioral difference is most visible near obstacles and at path corners: RPP follows the geometric path faithfully and slows when curvature is high; DWB may deviate from the geometric path when the local costmap provides better information.

---

## Equipment

- TurtleBot3 Burger with LDS-01/LDS-02 LiDAR
- Workstation with ROS2 Jazzy

---

## Instructions

### Preliminary: Environment Setup

Copy your `nav2_params.yaml`, map files, and filter masks from Project 8 into this repository. You will extend `nav2_params.yaml` throughout this project.

---

### Part 1 — Route Design

Before running any experiments, design your test route. This decision matters — a poor route will make Part 2 uninteresting.

**Requirements for your route:**

1. The path must be **at least 4 meters** in total length
2. The path must pass **near (within 0.5 m) but not through** your Project 8 keepout zone, so the inflation and keepout costs influence the planned path
3. The path must include **at least one corner or change of heading** greater than 45°, so the controller behavior is observable during the turn

Sketch your route on your map in RViz2, record the start and goal poses (map-frame x, y, yaw), and explain in your README *why* you chose this specific route — which constraint from the list above does each design choice satisfy, and why do you expect the keepout zone to influence the plan?

**Required evidence:**
- Annotated map screenshot showing the route with start, goal, and keepout zone visible
- Written route justification (3–5 sentences)
- Start and goal poses recorded as coordinates in your README

---

### Part 2 — Planner and Controller Comparison

Configure Nav2 to support both planners and both controllers simultaneously. You will run four combinations on your Part 1 route.

#### 2a. Configure both planners

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["Dijkstra", "AStar"]
    Dijkstra:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
    AStar:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

#### 2b. Configure both controllers

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["RPP", "DWB"]
    RPP:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.18
      lookahead_dist: 0.4
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      use_velocity_scaled_lookahead_dist: true
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
    DWB:
      plugin: "nav2_dwb_controller::DWBLocalPlanner"
      max_vel_x: 0.18
      min_vel_x: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

Select the planner and controller at runtime using `nav2_simple_commander`:

```python
nav.goToPose(goal_pose, planner_id='Dijkstra', controller_id='RPP')  # Run 1
nav.goToPose(goal_pose, planner_id='AStar',    controller_id='RPP')  # Run 2
nav.goToPose(goal_pose, planner_id='AStar',    controller_id='DWB')  # Run 3
```

#### 2c. Run all four combinations

Run your Part 1 route with each of the four planner × controller combinations:

| Run | Planner | Controller |
|-----|---------|------------|
| 1 | Dijkstra | RPP |
| 2 | A\* | RPP |
| 3 | A\* | DWB |

For each run, record:
- An RViz2 screenshot showing the **global plan** overlaid on the costmap (the `/plan` topic)
- Your observation of how the robot *drove* the path — did it track tightly? Cut the corner? Hesitate?

Write a `scripts/run_combinations.py` script that sends the goal to each of the three combinations in sequence. The script should wait for each run to complete before starting the next, and log the planner and controller used for each run.

**Required evidence:**
- 2 RViz2 screenshots of the planned path (one for Dijkstra, one for A\*; runs 2 and 3 share the same global plan — confirm this and explain why in your README)
- Written observations of controller behavior for RPP vs. DWB: what did you see that was different, and where on the route did it occur?
- `scripts/run_combinations.py` committed

---

### Part 3 — Analysis

Address all of the following in your README.

**On the planner comparison:**
- In runs 1 and 2, did Dijkstra and A\* produce identical paths, or different ones? If identical, explain why the keepout zone and inflation costs did not create a difference. If different, explain what specific cost structure in the costmap caused the divergence.
- A\* uses a heuristic to bias expansion toward the goal. On the route you designed, is the straight-line heuristic informative or misleading near the keepout zone boundary? Explain.

**On the controller comparison:**
- Describe one moment during your runs where RPP and DWB produced observably different behavior. Connect your description to the algorithmic difference: RPP is a geometric tracker, DWB is a scored trajectory sampler. Which property of each algorithm caused the behavior you observed?
- The `lookahead_dist` parameter in RPP controls the trade-off between tight path tracking and smooth motion. Based on what you saw, was 0.4 m a good choice for your route, or would you adjust it? Why?

---

## Deliverables

Submit by pushing to your team's **class GitHub repository** before the deadline.

### Repository Structure

```
proj9-path-planning/
├── README.md
├── maps/
│   ├── map_eerc722.yaml
│   └── map_eerc722.pgm
├── config/
│   ├── nav2_params.yaml          # Both planners and controllers configured
│   ├── keepout_mask.pgm
│   ├── keepout_mask.yaml
│   ├── speed_mask.pgm
│   └── speed_mask.yaml
├── scripts/
│   └── run_combinations.py       # Sends goal to all 4 planner × controller runs
└── figures/
    ├── route_annotated.png       # Annotated map with route and keepout zone
    ├── plan_dijkstra.png         # RViz2 plan for Dijkstra
    └── plan_astar.png            # RViz2 plan for A*
```

### README Requirements

#### 1. Route Design (8 pts)
- Annotated map screenshot
- Route justification (why this route, how it satisfies each design requirement)
- Start and goal poses

#### 2. Planner and Controller Comparison (27 pts)
- 2 RViz2 plan screenshots (Dijkstra and A\*)
- Explanation of whether/why the plans are identical or different
- Written observations of RPP vs. DWB controller behavior
- `run_combinations.py` committed and functional

#### 3. Analysis (11 pts)
- All four analysis questions answered with specific reference to your results

#### 4. Setup and Usage (2 pts)
- How to launch Nav2 with both planners and controllers configured
- How to select a specific planner and controller combination at runtime

---

## Grading Rubric

| Component                                                                               | Points |
| --------------------------------------------------------------------------------------- | ------ |
| **Route design:** annotated screenshot, written justification, keepout zone engagement  | 8      |
| **Planner comparison:** 2 plan screenshots, explanation of identical or different paths | 10     |
| **Controller comparison:** written behavioral observations connecting to algorithm      | 12     |
| **`run_combinations.py`:** functional, sends all 3 combinations                         | 5      |
| **Analysis:** all four questions answered with specific evidence                        | 11     |
| **Setup and usage instructions**                                                        | 2      |
| **Both team members have meaningful commits**                                           | 1      |
| **AI use documented**                                                                   | 1      |
| **Total**                                                                               | **50** |

---

## Tips

- **Run the planner comparison before the controller comparison.** Confirm that your two plans are what you expect before adding the controller variable.
- **Subscribe to `/plan` before sending the goal.** The plan is published once; you will miss it if you subscribe after the goal is sent.
- **For the controller comparison, watch the local plan in RViz2.** The DWB local plan (the red trajectory) updates in real time and shows which sampled trajectory was selected — this is what makes DWB's behavior visible.
- **Both planners must be listed in `planner_plugins` before launching Nav2.** You cannot add a planner at runtime without restarting the planner server. Same rule applies to `controller_plugins`.
- **Use floor tape to mark a consistent start pose.** AMCL drift will affect the planned path — set the 2D Pose Estimate to the same location for all four runs.

---

## References

1. Nav2 Planner Server Configuration: https://docs.nav2.org/configuration/packages/configuring-planner-server.html
2. Nav2 Controller Server Configuration: https://docs.nav2.org/configuration/packages/configuring-controller-server.html
3. Regulated Pure Pursuit Controller: https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
4. DWB Controller: https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html
5. TurtleBot3 Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/