# ROSbot + UAV

ROSbot is a differential-drive ground robot (ROS 2 Jazzy) with a small UAV mounted on top for aerial surveillance and inspection. The ground platform handles navigation, SLAM and obstacle avoidance while the UAV provides overhead imagery and telemetry via MAVLink (`mavros`).

## Features
- Differential drive with encoder feedback (N25 motors + L298N + Arduino Nano)  
- 2D LiDAR for SLAM (SLAM Toolbox) and Nav2 for navigation  
- UAV integration (PX4/ArduPilot + `mavros`) for aerial imaging  
- Works in Gazebo and on real hardware  
- Teleoperation and autonomous mission examples

## Requirements
- ROS 2 Jazzy  
- `nav2_bringup`, `slam_toolbox`, `mavros`  
- Arduino IDE or `arduino-cli` for uploading sketches  
- Gazebo (for simulation)

## Quick setup
```bash
# clone and build
git clone https://github.com/USERNAME/REPO.git
cd REPO
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

