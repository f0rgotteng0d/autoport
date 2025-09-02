# LDS02 Lidar Scanner ROS Package
## Overview
LDS02 is a 2D 360° Lidar scanner for robotics, SLAM, and autonomous navigation applications. It provides real-time environmental mapping using UART or USB and is directly supported in ROS.

## Features
 - 360° scanning for full-surround mapping.
 - Range: 160–8,000 mm, ±10 mm accuracy (close), up to ±5% (far).
 - Interfaces: UART/USB, 5V power supply.

## Usage

```
ros2 launch ld08_driver ld08.launch.py
```
