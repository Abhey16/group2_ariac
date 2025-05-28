# ENPM663 Final Project - Spring 2025
---


https://github.com/user-attachments/assets/a35e7208-2ad8-4722-a327-c5cc77418f94


---
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
- Detect and track parts (as mentioned in the order) in the bin using an RGB camera.
- Pick and place parts to the AGV by ceiling and floor robots.
- Send AGV with parts to the warehouse.
- End ARIAC competition.

# Prerequisites
- ROS 2 Iron Irwini (recommended)
- Gazebo (compatible with ARIAC)
- Docker (optional, if using containerized setup)
- ARIAC packages cloned and built in workspace

# launch
1. The robot_controller_node python node may not build depending on computer configuration.
    Run this from src/group2_ariac and rebuild if it did not build the first time
   ```
        chmod +x group2_ariac/group2_ariac/robot_controller_node.py
   ```
3. Build the workspace:
   ```
    colcon build --symlink-install
    source install/setup.bash
   ```

4. Launch ARIAC
    ```
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=group2_ariac trial_name:=final_project
    ```
6. Run nodes
    ```
    ros2 launch group2_ariac group2_ariac.launch.py
    ```
