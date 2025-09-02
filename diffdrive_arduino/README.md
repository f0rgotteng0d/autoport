# diffdrive_arduino
## Overview
diffdrive_arduino is a ROS2 hardware interface node designed to work with an Arduino running the ros_arduino_bridge firmware. It provides a bridge between the Arduino motor controller and the ROS2 control ecosystem by implementing a ros2_control hardware interface compatible with the diff_drive_controller.

This package enables velocity control and position/velocity feedback for a differential drive robot with two independently controlled motors.

## Features
- Supports two motor velocity controls.
- Provides encoder position and velocity feedback.
- Communicates with Arduino over serial (configurable port and baud rate).
- Designed to be used with the ROS2 diff_drive_controller from ros2_control.
- Real-time and reliable hardware interface for differential drive robots.

## Requirements
- Arduino device running [ros_arduino_bridge](https://github.com/joshnewans/ros_arduino_bridge.git) firmware.
- ROS2 (compatible with ros2_control framework).
- Serial connection to Arduino.

## Attribution

This library was originally developed and maintained by [Josh Newans](https://github.com/joshnewans)
. Full source code and documentation are available in the [original repository](https://github.com/joshnewans/diffdrive_arduino?tab=readme-ov-file)


