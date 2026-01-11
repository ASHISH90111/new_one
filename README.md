# Custom A* Global Planner for ROS 2 Nav2

## Overview
This package implements a **custom global planner based on the A\* algorithm** and integrates it into the **ROS 2 Nav2 navigation stack** as a plugin.  
The planner is designed as a **standalone ROS 2 package**, compatible with Nav2’s global planner interface, and demonstrates end-to-end navigation on a **TurtleBot mobile robot in Gazebo**, with goals provided via **RViz**.

The implementation follows ROS 2 and Nav2 best practices and does **not modify any Nav2 source code**.

---

## Features
- Custom A* global planner implemented as a Nav2 plugin
- Euclidean distance (L2 norm) heuristic
- Diagonal neighbor expansion
- Costmap-aware planning using Nav2 costmaps
- Integrated with Nav2 Behavior Tree and controller stack
- End-to-end navigation in Gazebo using TurtleBot
- RViz visualization and goal input

---

## Technical Stack
- **ROS 2:** Humble Hawksbill  
- **Navigation Stack:** Nav2  
- **Simulation:** Gazebo  
- **Robot Model:** TurtleBot  
- **Visualization:** RViz  

---

## Planner Algorithm

### A* Search
The planner uses the A* search algorithm on a 2D grid derived from the Nav2 global costmap.

Each node maintains:
- Grid coordinates `(x, y)`
- Cost from start `g(n)`
- Heuristic cost to goal `h(n)`
- Total cost `f(n) = g(n) + h(n)`

Nodes are expanded based on the minimum `f(n)` value until the goal is reached.

---

### Heuristic (Euclidean Distance)
The heuristic function is defined as:
h(n) = sqrt((x_current - x_goal)² + (y_current - y_goal)²)

---

Properties:
- Admissible (never overestimates the true cost)
- Consistent
- Suitable for grid-based planning with diagonal motion

---

### Motion Model
- 8-connected grid (cardinal + diagonal neighbors)
- Cardinal movement cost: `1.0`
- Diagonal movement cost: `sqrt(2)`
- Obstacle checking is performed using the Nav2 global costmap

---

## Integration with Nav2

### Planner Plugin
- Implements `nav2_core::GlobalPlanner`
- Registered using `pluginlib`
- Outputs `nav_msgs::msg::Path`, fully compatible with Nav2
- Consumes Nav2 global costmap data directly

---

### Nav2 Configuration
- Default Nav2 global planner (`GridBased`) is disabled
- Custom planner (`CustomAStar`) is enabled via YAML configuration
- Behavior Tree XML is customized to reference `planner_id="CustomAStar"`
- No Nav2 source code is modified

---

### Navigation Pipeline
1. Goal is sent from RViz
2. Behavior Tree triggers global path planning
3. Custom A* planner computes a global path
4. Nav2 controller stack follows the path
5. Robot navigates to the goal in Gazebo

---

## Running the Demo

### 1. Build the Workspace
```bash
cd ~/nav2_ws
colcon build --symlink-install
source install/setup.bash
```
---
### 2. Launch Gazebo
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
---

### 3. Launch Nav2 with Custom Planner
```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml \
  params_file:=/home/abhi/nav2_ws/src/custom_astar_planner/config/nav2_params.yaml
```
---

### 4. RViz
```bash
rviz2
```
---

## Steps in RViz:

1. Set Fixed Frame to map

2. Use 2D Pose Estimate to initialize localization

3. Use 2D Nav Goal to send navigation goals

## Tunable Parameters

Planner and navigation behavior can be adjusted using parameters in nav2_params.yaml, including:

- Costmap resolution

- nflation radius

- Planner frequency

- Controller (DWB) parameters


These parameters influence:

- Path smoothness

- Obstacle clearance

- Navigation stability


## Assumptions & Limitations

Planning is performed on a 2D grid (no elevation handling)

Path smoothing is handled by Nav2’s controller and smoothing stack

Diagonal corner-cutting prevention can be further improved in dense environments

Performance is suitable for typical AMR-scale maps; very large maps may require optimization

## Results

Obstacle-aware global paths are generated successfully

Seamless integration with Nav2 controller and recovery behaviors

Stable end-to-end navigation demonstrated in Gazebo

Navigation goals from RViz are executed without manual intervention

## Conclusion

This project demonstrates the design, implementation, and integration of a custom A* global planner within the ROS 2 Nav2 framework.
It reflects real-world AMR navigation workflows, emphasizing correctness, modularity, and engineering best practices.
