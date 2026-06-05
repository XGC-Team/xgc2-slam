# xgc2-slam

Private ROS1 Noetic SLAM package repository for XGC2.

## Packages

- `fast_lio`: FAST-LIO2 ROS package. The Deb artifact is named `ros-noetic-xgc2-fast-lio2`.
- `swarm_lio`: Swarm-LIO2 ROS package. The Deb artifact is named `ros-noetic-xgc2-swarm-lio2`.
- `point_lio`: Point-LIO ROS package. The Deb artifact is named `ros-noetic-xgc2-point-lio`.
- `lio_sam`: LIO-SAM ROS package. The Deb artifact is named `ros-noetic-xgc2-lio-sam`.
- `voxel_slam`: Voxel-SLAM ROS package. The Deb artifact is named `ros-noetic-xgc2-voxel-slam`.
- `voxelslam_pointcloud2`: Voxel-SLAM RViz point cloud plugin. The Deb artifact is named `ros-noetic-xgc2-voxelslam-pointcloud2`.
- `livox_ros_driver`, `swarm_msgs`, and `udp_bridge`: support packages required by the bundled SLAM stacks.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-slam
```

The `ros-noetic-xgc2-slam` meta package pulls in FAST-LIO2, Swarm-LIO2, Point-LIO, LIO-SAM, Voxel-SLAM, the Voxel-SLAM RViz plugin, and their support packages.

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
rospack find fast_lio
rospack find swarm_lio
rospack find point_lio
rospack find lio_sam
rospack find voxel_slam
rospack find voxelslam_pointcloud2
```
