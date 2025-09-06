# rplidar_ros
## Overview
The rplidar_ros package provides ROS integration for the RPLIDAR sensor, enabling real-time 360-degree 2D laser scanning capabilities. It publishes sensor data to ROS topics for use in mapping, localization, and navigation applications.

# Features
- Supports RPLIDAR A-series sensors with high-speed laser scanning
- Publishes LaserScan messages compatible with ROS navigation stack
- Configurable parameters for frame id, scanning mode, and baud rate
- Includes launch files for easy sensor initialization

## Usage
Launch the driver with a sample launch file:
```
ros2 launch rplidar_ros rplidar_a2m8.launch.py
```
The package publishes scan data to the /scan topic for downstream nodes to consume.

## Parameters
- frame_id: TF frame of the laser (default: "laser")
- serial_port: Device port of the RPLIDAR (default: "/dev/ttyUSB0")
- serial_baudrate: Baud rate for serial communication (default: 115200)
- angle_compensate: Enable angle compensation (default: true)

  ## Attribution

  This library was originally developed and maintained by [Slamtec](https://github.com/Slamtec) . Full source code and documentation are available in the [original repository](https://github.com/Slamtec/rplidar_ros).
