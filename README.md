# xgc2-slam

Private ROS1 Noetic SLAM package repository for XGC2.

This tree only aggregates already-packaged algorithms. It does not run a
combined CI matrix. Debug and CI each algorithm in its own XGC-Team
repository. Vehicle-specific work uses a `robot-sensor` branch on that
fork (for example `scout-helios16`). The release workflow here is only
for Debian artifacts that are already published.

## Packages

- `fast_lio`: FAST-LIO2 ROS package. The Deb artifact is named `ros-noetic-xgc2-fast-lio2`.
- `faster_lio`: Faster-LIO ROS package, synced from the `XGC-Team/xgc2-faster-lio` fork. The Deb artifact is named `ros-noetic-xgc2-faster-lio`.
- `swarm_lio`: Swarm-LIO2 ROS package. The Deb artifact is named `ros-noetic-xgc2-swarm-lio2`.
- `point_lio`: Point-LIO ROS package. The Deb artifact is named `ros-noetic-xgc2-point-lio`.
- `lio_sam`: LIO-SAM ROS package. The Deb artifact is named `ros-noetic-xgc2-lio-sam`.
- `voxel_slam`: Voxel-SLAM ROS package. The Deb artifact is named `ros-noetic-xgc2-voxel-slam`.
- `voxelslam_pointcloud2`: Voxel-SLAM RViz point cloud plugin. The Deb artifact is named `ros-noetic-xgc2-voxelslam-pointcloud2`.
- `swarm_msgs` and `udp_bridge`: support packages required by the bundled SLAM stacks.

Livox support is provided by the separate driver product package `ros-noetic-livox-ros-driver`.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-slam
```

The `ros-noetic-xgc2-slam` meta package pulls in FAST-LIO2, Faster-LIO, Swarm-LIO2, Point-LIO, LIO-SAM, Voxel-SLAM, the Voxel-SLAM RViz plugin, and their support packages.

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
rospack find fast_lio
rospack find faster_lio
rospack find swarm_lio
rospack find point_lio
rospack find lio_sam
rospack find voxel_slam
rospack find voxelslam_pointcloud2
```
