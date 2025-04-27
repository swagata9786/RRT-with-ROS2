# RRT-with-ROS2
A C++ implementation of the RRT* (Rapidly-Exploring Random Tree Star) path planning algorithm for ROS2 humble with OpenCV visualization. Supports dynamic start/goal selection, grid generation, and smooth path generation.
## Table of Contents
- [Prerequisities](#prequisities)
## Prerequisites
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
cd /rrt_star_planner/src
git clone https://github.com/your-username/your-rrt-star-repo.git
