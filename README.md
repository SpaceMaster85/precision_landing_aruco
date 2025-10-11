# Precision Landing with ROS2, PX4, and ArUco Markers

This project demonstrates the autonomous landing of a drone using ArUco markers, ROS2, and PX4. It features vision-based target detection, approach, and landing with configurable controllers, NED transformations, and spiral waypoint generation for robust target acquisition.

---

## 📦 Features

* **Vision-based Target Detection**: Utilizes ArUco markers for target identification.
* **Spiral Waypoint Generation**: Creates a spiral flight path for target approach.
* **NED Transformations**: Converts coordinates between different reference frames.
* **Configurable Controllers**: Adjustable PID controllers for precise control.
* **PX4 Integration**: Seamless communication with PX4 via ROS2.

---

## 🛠 Installation

### 1. Prerequisites

* Ubuntu 22.04
* ROS2 Humble
* PX4 Autopilot with an ArUco marker and downward-facing camera
* Micro XRCE-DDS Agent
* QGroundControl Daily Build
* OpenCV 4.10.0
* ROS_GZ bridge

For detailed setup instructions, refer to the following resources:

* [PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html)
* [QGroundControl Daily Builds](https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html)
* [ros_gz Bridge](https://github.com/gazebosim/ros_gz)

### 2. Clone the Repository

```bash
git clone https://github.com/SpaceMaster85/precision_landing.git
cd precision_landing
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Build the Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s ~/precision_landing .
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

### 1. Launch PX4 Simulation

```bash
make px4_sitl_default gazebo
```

### 2. Start ROS2 Node

```bash
ros2 run precision_landing landing_node
```

### 3. Provide ArUco Markers

Ensure that the ArUco markers are visible and detected by the camera.

---

## 🧪 Testing

Use the provided launch files to test the system with simulated data.

---

## 📄 License

This project is licensed under the BSD-3-Clause License.

#### Thank you
https://github.com/ARK-Electronics/tracktor-beam
