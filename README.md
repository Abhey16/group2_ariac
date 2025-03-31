# ENPM663 RWA #2 - Spring 2025
**Group Members:**  
- Rey (120498626)
- Wei-Li Chen  (120378508)
- Abhey  

# Overview

This project is part of **ENPM663: Building a Manufacturing Robotic Software System**, RWA #2.  
The goal is to manage an ARIAC competition environment using **ROS 2** and **Gazebo**, with focus on the following tasks:

- Start ARIAC competition.
- Receive and store orders, extract important info.
- Due to limited time and knowledge, **skiped Fulfill Order**.
- Complete order by shipping orders to warehouse with AGVs.
- End ARIAC competition.

# Prerequisites
- ROS 2 Iron Irwini (recommended)
- Gazebo (compatible with ARIAC)
- Docker (optional, if using containerized setup)
- ARIAC packages cloned and built in workspace

# launch
1. Build the workspace:
    colcon build --packages-select ariac_gazebo
    source install/setup.bash
2. Launch ARIAC
    ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa2_spring2025
3. Run nodes
    ros2 launch group2_ariac group2_ariac.launch.py
