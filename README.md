A C++ implementation of the RRT* (Rapidly-Exploring Random Tree Star) path planning algorithm for ROS2 humble with OpenCV visualization. Supports dynamic start/goal selection, grid generation, and smooth path generation.
## Table of Contents
- [Prerequisities](#prequisities)
- [Features](#features)
- [Nodes and Topics](#nodesandtopics)
- [Packages](#packages)
- [Build Guide](#buildguide)
- [How to Run](#howtorun)
- [Output](#output)
## Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble [(Required Package)](https://docs.ros.org/en/humble/Package-Docs.html?utm_source=chatgpt.com)
  + OpenCV
## Features
- 2D grid environment 
- Start and goal selection and grid generation via command
- Path smoothing for better accuracy
- Fast and optimized tree expansion
## Nodes and Topics
![image](https://github.com/user-attachments/assets/25cb0ad3-c159-4ece-b668-166a497e51c0)
## Packages
- user_input_node: Handles user given start, goal, and grids
- planner_node: Computes the RRT* tree and finds optimal paths
- visualizer_node: Draws the grid, tree, and final path in real-time
## Build Guide 
- Clone the Repository
First, navigate to your ROS 2 workspace and clone the project:
```
cd /rrt_star_planner/src
git clone https://github.com/swagata9786/rrtstar-with-ros2.git
```
## How to Run
- Using a Launch File
Launch the RRT* node with predefined parameters:
```
ros2 launch rrt rrt_start.launch.py
```
This launch file automatically sets important parameters like grid size, start cell, and goal cell.

- Running Manually with Custom Parameters
Alternatively, runs the RRT* node directly and manually specify parameters:
```
ros2 run rrt rrt_star_node --ros-args \
-p start_cell_x:=3 \
-p start_cell_y:=2 \
-p goal_cell_x:=24 \
-p goal_cell_y:=48 \
-p rows:=50 \
-p cols:=30
```
- rows and cols define the size of the grid (50 rows × 30 columns).
- start_cell_x, start_cell_y define the start position at cell (3, 2).
- goal_cell_x, goal_cell_y define the goal position at cell (29, 42).
## Output
![WhatsApp Image 2025-04-27 at 11 03 00](https://github.com/user-attachments/assets/13d3be5e-048d-4582-8596-845cf1e26f70)
![WhatsApp Image 2025-04-27 at 14 34 10](https://github.com/user-attachments/assets/172245a5-1337-4f9f-a279-117ddeeb3e93)
![Screenshot from 2025-04-27 15-09-38](https://github.com/user-attachments/assets/3d36ca55-49de-4eac-b44b-9dd25e5d9777)
![Screenshot from 2025-04-27 15-09-54](https://github.com/user-attachments/assets/371a50ec-a3da-47db-a493-95d3aba0c846)






