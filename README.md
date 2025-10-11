# Precision Landing with ROS2, PX4, and Pose Messages

This project demonstrates autonomous precision landing of a drone using Pose Messages, leveraging ROS2 and PX4. It provides vision-based target search, approach, and landing with configurable controllers, NED transformations, and spiral waypoint generation for robust target acquisition.

## 🚀 Features

- **Vision-Based Target Detection:** Use of ArUco markers for precise landing target identification.
- **Autonomous Landing:** Integration with ROS2 and PX4 for autonomous marker-based landing.
- **Configurable Controllers:** Adjustable PID controllers for fine landing control.
- **Spiral Waypoints:** Generates spiral waypoints for safe target approach.
- **NED Transformations:** Converts between local and global coordinate systems for precise navigation.

## 📦 Prerequisites

- **PX4 Autopilot:** Installed and configured on the drone.
- **ROS2 Foxy Fitzroy or later:** For communication and control.
- **Git:** For cloning the repository and managing submodules.

## 🔧 Installation

### 1. Setup Enviroment

For detailed setup instructions, refer to the following resources:

* [PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html)
* [QGroundControl Daily Builds](https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html)
* [ros_gz Bridge](https://github.com/gazebosim/ros_gz)

### 2. Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SpaceMaster85/precision_landing.git
cd precision_landing
```

### 3. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

### 1. Launch PX4 Simulation


Launch PX4 Simulation
```bash
make px4_sitl gz_x500_mono_cam_down_aruco
```
Launch MicroXRCEAgent

```bash
micro-xrce-dds-agent udp4 -p 8888
```
Launch the ros_gz_bridge to bridge the camera image topic from Gazebo to ROS2
```bash
ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image
```
Launch the ros_gz_bridge to bridge the camera info topic from Gazebo to ROS2 (this is how we get camera intrinsics)
```bash
ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### 2. Start ROS2 Node

```bash
ros2 run precision_landing landing_node
```

### 3. Provide Pose Messages

Ensure that the Pose Messages are provided to the ROS topic /target_pose
I recommend to use ARUCO markers. See https://github.com/SpaceMaster85/aruco_id_checker and https://github.com/namo-robotics/aruco_markers

---

## 🧪 Testing

Use the provided launch files to test the system with simulated data.

---

## 📄 License

This project is licensed under the BSD-3-Clause License.

#### Thank you
https://github.com/ARK-Electronics/tracktor-beam
