# lucia_cartographer
[![ROS 2 Distro - Humble](https://img.shields.io/badge/ros2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS2 Distro - Jazzy](https://img.shields.io/badge/ros2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
## 🚀 Overview
This package provides launch files and minimal configuration to run `slam_toolbox` on the Lucia for real-time mapping (SLAM).
It assumes Lucia’s motors and LiDAR (e.g., urg_node2) are already running and publishing scans.

## 🛠️ Setup
### Install cartographer
```bash
sudo apt install ros-$ROS_DISTRO-cartographer
sudo apt install ros-$ROS_DISTRO-cartographer-ros
```
### Clone & Build
```bash
cd ~/ros2_ws/src  #Go to ros workspace
git clone https://github.com/iHaruruki/lucia_cartographer.git #clone this package
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
> [!NOTE]
> Make sure related packages (e.g., `lucia_controller`, `lucia_description`,`urg_node2`) are built 
> in the same workspace and sourced as well.

## 🎮 Usage
### Launch Lucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
### Run Cartographer Node
```shell
ros2 launch lucia_cartographer cartgrapher.launch.py
```
### Run Teleportation Node
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**When you move Lucia, the map will update.**
![](media/cartographer.gif)
### Save map
```shell
# Once the entire map is complete, save it
ros2 run nav2_map_server map_saver_cli -f ~/map
```
## 📜 License
## 👤 Authors
- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 📚 References
