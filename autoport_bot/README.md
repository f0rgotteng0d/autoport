# Autoport
## Overview
This ROS2 package contains the core robot description, configurations, launch files, and Gazebo simulation environments for a differential drive robot system. It provides everything needed to simulate, control, and visualize the robot in a modular manner.

# File Structure
```
.
├── CMakeLists.txt
├── config
│   ├── gazebo_params.yaml
│   ├── mapper_params_online_async.yaml
│   └── my_controllers.yaml
├── description
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── lidar.xacro
│   ├── robot_core.xacro
│   ├── robot.urdf.xacro
│   └── ros2_control.xacro
├── launch
│   ├── launch_robot.launch.py
│   ├── launch_sim.launch.py
│   └── rsp.launch.py
├── package.xml
```
## Features

- Robot description using URDF and xacro macros.
- Configurable control parameters and Gazebo simulation settings.
- Launch files for running the robot and simulation with ROS2.
- Sample empty Gazebo world for testing.

## Usage

Launch the real robot:
```
ros2 launch autoport_bot launch_robot.launch.py
```

Launch the simulation:
```
ros2 launch autoport_bot launch_sim.launch.py
```

