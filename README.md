# lucia_cartographer
[![ROS 2 Distro - Humble](https://img.shields.io/badge/ros2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS2 Distro - Jazzy](https://img.shields.io/badge/ros2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
## 🚀 Overview
Launch files and minimal configuration for running Cartographer on the Lucia.

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

### Check map
```bash
eog ~/map.pgm
```

## 🧯 Troubleshooting
| Symptom | Likely Cause | Fix |
| ------- | ------------ | --- |
| Map does not grow | No/slow scans on /scan | Check `ros2 topic hz /scan` (~10 Hz recommended). Verify remapping. |
| Map warps while moving | Bad TF or wrong sensor pose | Check `ros2 run tf2_tools view_frames`. Fix static transforms and sensor mounting. |
| Map saving fails | Path/permission issues | Ensure the `-f` path is writable; expect `.pgm` and `.yaml` outputs. |
| Nothing shows in RViz | Wrong Fixed Frame | Set Fixed Frame to `map`; verify TF and LaserScan topic. |

## 📜 License
## 👤 Authors
- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 📚 References
