# Autoport -  A SLAM Bot
SLAM-enabled autonomous robot that builds real-time maps and localizes itself for navigation in unknown environments.

## Table of Contents
- [About the Project](#about-the-project)
  - [Tech Stack](#tech-stack)
- [Getting Started](#getting-started)
  - [Prerequisites](#Prerequisites)
  - [Installation](#Installation)
- [Usage](#usage)
- [Future Work](#future-work)
- [Troubleshooting](#troubleshooting)
- [Contributors](#contributors)
- [Acknowledgements and Resources](#acknowledgements-and-resources)

## About the Project 
Autoport is a ROS 2-powered mobile robot designed for autonomous exploration, mapping, and navigation using Simultaneous Localization and Mapping (SLAM).
Built on affordable hardware like the Raspberry Pi and common sensors, Autoport serves as a platform for robotics research, education, and prototyping.

### Tech Stack
- [ROS2](https://docs.ros.org/en/rolling/index.html)
- [SLAM](https://www.mathworks.com/discovery/slam.html)
- [Raspberry Pi 4/5](https://www.raspberrypi.com/) or similar
- Arduino compatible hardware e.g. [Arduino Nano](https://docs.arduino.cc/hardware/nano/) or [ESP32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf)


## Getting Started
### Prerequisites
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
### Installation
  ```
  git clone https://github.com/f0rgotteng0d/autoport.git
  ```

## Usage

In different terminals, run the following commands -
```
ros2 launch autoport_bot launch_robot.launch.py
ros2 launch ld08_driver ld08.launch.py
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/autoport_bot/config/mapper_params_online_async.yaml use_sim_time:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

```

## Results and Demos

Using real-time 2D SLAM and laser scanner data, an accurate occupancy grid map of an indoor environment was generated. The map below demonstrates the quality and detail achievable in typical room settings.
### SLAM Map Example
<img width="1920" height="1080" alt="Screenshot from 2025-09-06 16-27-46" src="https://github.com/user-attachments/assets/64fa68c9-5182-412e-817d-4c6a322533ec" /><br><br>

Watch this simulation showcasing the Nav2 navigation stack in action for autonomous robot path planning and obstacle avoidance.<br>
[Watch the video](https://youtu.be/KvBy6v8WLM0)



## Future Work
- [ ] Add support for a camera onboard to increase accuracy
- [ ] Expand environment types to adapt to outdoors or tricky terrain
- [ ] Improve power efficiency
- [ ] Make another custom bot and enable collaborative SLAM

## Troubleshooting

- **Cannot connect to serial port**
  - Ensure the serial device path is correct (`/dev/ttyUSB0` or equivalent).
  - Verify you have permission to access the device (try `sudo` or add your user to the `dialout` group).
  - Confirm the cable and hardware connections are secure.

- **Motors stop working**
   - Check your motor driver is functional and outputs proper voltage.
   - Verify your connections and try changing cables.
   - Make sure battery is properly charged.

- **No data received from Lidar or motors**
  - Check that the device is powered and turned on.
  - Verify baud rate and serial parameters match device specs.
  - Inspect wiring and connectors for faults.

- **ROS node fails to start or crashes**
  - Confirm all ROS dependencies are installed.
  - Verify environment is sourced (`source install/setup.bash`).
  - Check logs for errors and missing configurations.

- **Communication timeouts or errors**
  - Increase serial timeout settings in parameters.
  - Close other applications that might be using the serial port.

## Contributors
- [Kushaan Gada](https://github.com/f0rgotteng0d)
- [Hrishi Pandey](https://github.com/Hrishi010905)
- [Rayon Biswas](https://github.com/RayonBiswas)

## Acknowledgements and Resources 
- [SRA VJTI](https://sravjti.in/) Eklavya 2025
- Github Repositories we refered to [JoshNewans](https://github.com/joshnewans/articubot_one)
- [Articulated Robotics](https://articulatedrobotics.xyz/)
 


