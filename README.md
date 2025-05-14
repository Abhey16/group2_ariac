# ENPM663 RWA #3 - Spring 2025
**Group Members:**  
- Rey Roque (120498626)
- Wei-Li Chen  (120378508)
- Abhey  (120110306)

# Overview

This project is part of **ENPM663: Building a Manufacturing Robotic Software System**, Final Project.  
The goal is to manage an ARIAC competition environment using **ROS 2** and **Gazebo**, with focus on the following tasks:

- Start ARIAC competition.
- Receive and parse orders.
- Detect trays using ArUco markers and estimate their poses.
- Detect and track parts (**purple pumps** and **blue batteries**) moving along the conveyor using an RGB camera.
- Predict future poses of moving conveyor parts based on belt speed.
- Log the required tray and part information **only once** per item.
- End ARIAC competition.

# Prerequisites
- ROS 2 Iron Irwini (recommended)
- Gazebo (compatible with ARIAC)
- Docker (optional, if using containerized setup)
- ARIAC packages cloned and built in workspace

# launch
1. Build the workspace:
    colcon build --symlink-install
    source install/setup.bash

2. Launch ARIAC
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=group2_ariac trial_name:=final_project

3. Run nodes
    ros2 launch group2_ariac group2_ariac.launch.py

4. The robot_controller_node python node may not build depending on computer configuration.
    Run this from src/group2_ariac and rebuild if it did not build the first time
        chmod +x group2_ariac/group2_ariac/robot_controller_node.py