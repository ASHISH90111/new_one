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

