# xgc2-slam

Private ROS1 Noetic SLAM package repository for XGC2.

## Packages

- `fast_lio`: FAST-LIO2 ROS package. The Deb artifact is named `ros-noetic-xgc2-fast-lio2`.
- `swarm_lio`: Swarm-LIO2 ROS package. The Deb artifact is named `ros-noetic-xgc2-swarm-lio2`.
- `livox_ros_driver`, `swarm_msgs`, and `udp_bridge`: support packages required by the two LIO stacks.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-slam
```

The `ros-noetic-xgc2-slam` meta package pulls in FAST-LIO2, Swarm-LIO2, and their support packages.

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
rospack find fast_lio
rospack find swarm_lio
```
