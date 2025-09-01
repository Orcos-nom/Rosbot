# ROSbot + UAV — Ground Robot with Aerial Surveillance

**ROSbot** is a differential-drive ground robot built using **ROS 2 Jazzy**. It is equipped with a 2D LiDAR for SLAM and navigation and uses an Arduino for motor control. A **UAV (drone)** is mounted on top to provide **aerial surveillance**, giving a combined ground and aerial view of the environment.

---

## 📖 Project Overview
This project integrates a mobile ground robot and a UAV into a single platform. The ROSbot handles ground navigation, obstacle avoidance, and mapping, while the drone enables aerial surveillance for extended situational awareness. The system is designed for both **simulation in Gazebo** and **deployment on real hardware**.

---

## ✨ Features
- Differential-drive ground platform  
- 2D LiDAR for SLAM and autonomous navigation (SLAM Toolbox + Nav2)  
- Arduino Nano for motor & encoder interface  
- UAV mounted for aerial surveillance  
- Works in both Gazebo simulation and on physical hardware  

---

## 🚁 Drone
- A quadcopter is mounted on the ROSbot.  
- Purpose: **Aerial surveillance** to complement ground navigation.  

---

## 🔧 Hardware (Ground Robot)
- **Motors:** 2 × N25 DC motors with encoders  
- **Motor Driver:** L298N  
- **Controller:** Arduino Nano  
- **Sensors:** 2D LiDAR (e.g., RPLIDAR)  
- **Other:** Caster wheel, battery/power system  

---

## 🖥️ Simulation
- Gazebo for robot and UAV simulation  
- PX4/ArduPilot SITL for UAV (optional)  
- Test SLAM, navigation, and UAV surveillance in virtual environments  

---
## **License**  

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## ⚡ Quick Setup
```bash
# clone and build
git clone https://github.com/Orcus-nom/REPO.git
cd REPO
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
## 👨‍💻 Authors

- **Shadab Ahmad Khna** – Developer & Maintaine  
- Simulation Support  
- Hardware Integration  
- Documentation
- **mail** -shadabahmadkhan272@gmail.com 

