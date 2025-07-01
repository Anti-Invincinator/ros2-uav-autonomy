# 🛩️ ROS2 UAV Autonomy Stack

A modular and extensible UAV autonomy framework built on **ROS2 Foxy**, integrated with **PX4**, **Gazebo SITL**, and **robot_localization** for EKF-based state estimation. Designed for simulation-driven development of fully autonomous drones with mission planning, sensor fusion, and AI perception.

---

## 🚀 Features Implemented

### ✅ System Foundation
- [x] Ubuntu 20.04 VM with full-screen + Guest Additions
- [x] ROS2 Foxy installed and workspace configured
- [x] PX4 SITL with Gazebo Classic
- [x] VS Code integration for dev and debugging

### ✅ Core ROS2 Packages
- **`sensors/`**
  - Publishes simulated GPS (`/gps/fix`) and IMU (`/imu/data`)
- **`fusion/`**
  - Uses `robot_localization` EKF to fuse GPS + IMU → `/odometry/filtered`
  - Configurable launch file for real-time localization

---

## 📁 Project Structure
uav_ws/
├── src/
│ ├── sensors/ # Simulated sensor publishers
│ ├── fusion/ # EKF-based localization
│ ├── launch/ # Centralized launch control (WIP)
│ └── planner/ # (To be added) Mission and behavior logic


---

## 🧱 Tech Stack

| Component         | Tech                              |
|------------------|-----------------------------------|
| OS               | Ubuntu 20.04 LTS                  |
| Middleware       | ROS2 Foxy                         |
| Autopilot        | PX4 (SITL)                        |
| Simulator        | Gazebo Classic                    |
| Sensor Fusion    | robot_localization (EKF)          |
| IDE              | Visual Studio Code                |
| Visualization    | RViz2 (planned)                   |
| Control Bridge   | MAVROS2 (planned)                 |
| Perception       | CNN + camera image pipeline (WIP) |

---

## 🚧 Roadmap & Checklist

### 🧩 Sensor Expansion
- [ ] Barometer data publisher
- [ ] Magnetometer (`/mag/data`) simulation
- [ ] Depth/Range sensor mock node
- [ ] Visual Odometry (VIO) or Optical Flow integration

### 🛠️ Control & Integration
- [ ] Integrate MAVROS2 for PX4 communication
- [ ] Enable offboard control via ROS2 `/setpoint_position/local`
- [ ] Arm/takeoff/land mission services

### 🧭 Navigation & Behavior
- [ ] `planner/` package:
  - [ ] Multi-waypoint mission planner
  - [ ] Reactive behavior node (avoidance, hover, return-home)
- [ ] Behavior state machine (e.g., FSM or Behavior Trees)

### 📦 Visualization
- [ ] RViz2 launch with:
  - [ ] `/odometry/filtered` path display
  - [ ] TF tree
  - [ ] Real-time GPS, IMU, and velocity overlays

### 🎯 Perception
- [ ] Add camera sensor to Gazebo model
- [ ] `vision/` package with CNN-based object detection
- [ ] Integrate OpenCV + PyTorch for real-time detection
- [ ] Detected object → command callback (e.g., fly-to or avoid)

### 🤖 AI + ML Modules
- [ ] YOLOv5/YOLO-NAS or MobileNetV2 inference pipeline
- [ ] Use image frames from Gazebo camera
- [ ] Process frames and publish results to ROS2 topic

### 📤 Deployment & Tooling
- [ ] Launch all modules from one unified file
- [ ] Log data to ROS2 bag files for evaluation
- [ ] Modular config files for different drones and sensors
- [ ] Unit tests and launch tests using `ament_lint` and `pytest`

---

## 🧠 How to Launch the Current System

```bash
# Source the workspace
source ~/uav_ws/install/setup.bash

# Start sensors
ros2 launch sensors sensors_launch.py

# Start EKF localization
ros2 launch fusion ekf_launch.py

# (Optional) Launch Gazebo + PX4
cd ~/PX4-Autopilot
make px4_sitl_default gazebo

