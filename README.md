# Autoport -  A SLAM Bot
SLAM-enabled autonomous robot that builds real-time maps and localizes itself for navigation in unknown environments.

# About the Project 
Autoport is a ROS 2-powered mobile robot designed for autonomous exploration, mapping, and navigation using Simultaneous Localization and Mapping (SLAM).
Built on affordable hardware like the Raspberry Pi and common sensors, Autoport serves as a platform for robotics research, education, and prototyping.

# Tech Stack
- [ROS2](https://docs.ros.org/en/rolling/index.html)
- [SLAM](https://www.mathworks.com/discovery/slam.html)
- [Raspberry Pi 4/5](https://www.raspberrypi.com/) or similar

# File Structure
# Getting Started
## 1.Prerequisites
- Hardware
  - Raspberry Pi 4/5 (recommended) or Jetson Nano
  - LIDAR (e.g., RPLidar A1/A2, LDS08 Lidar, Hokuyo)
  - IMU (ISM330DHCX / MPU6050 / BNO055 / etc.)
  - Motor driver (L298N / TB6612FNG / ODrive)
  - Differential drive base

- Software
  - Ubuntu 22.04 or 24.04 (or supported ROS 2 distro)    
  - ROS 2 Humble / Jazzy
      You can visit the official [ROS Site](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to install ROS.
  - slam_toolbox or cartographer
  - 
    ```
    #Update apt and install slam_toolbox
      sudo apt update
      sudo apt install ros-<ROS_DISTRO>-slam-toolbox
    
    #Source the ROS environment
      source /opt/ros/<ROS_DISTRO>/setup.bash
    
    #(Optional) Verify package executables
      ros2 pkg executables slam_toolbox
    ```
  - ros2_control + nav2

    ```
    #Install ros2_control
      sudo apt update
      sudo apt install -y \
      ros-$ROS_DISTRO-ros2-control \
      ros-$ROS_DISTRO-ros2-controllers \
      ros-$ROS_DISTRO-controller-manager \
      ros-$ROS_DISTRO-hardware-interface \
      ros-$ROS_DISTRO-transmission-interface

    #Install Nav2
      sudo apt update
      sudo apt install -y \
      ros-$ROS_DISTRO-nav2-bringup \
      ros-$ROS_DISTRO-navigation2 \
      ros-$ROS_DISTRO-nav2-map-server \
      ros-$ROS_DISTRO-nav2-amcl \
      ros-$ROS_DISTRO-nav2-lifecycle-manager


    
    ```

# Contributors
- [Kushaan Gada](https://github.com/f0rgotteng0d)
- [Hrishi Pandey](https://github.com/Hrishi010905)
- [Rayon Biswas](https://github.com/RayonBiswas)

# Acknowledgements and Resources 
- [SRA VJTI](https://sravjti.in/) Eklavya 2025
- Github Repositories we refered to [JoshNewans](https://github.com/joshnewans/articubot_one)
- [Articulated Robotics](https://articulatedrobotics.xyz/)
 


