#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-slam >/dev/null
dpkg -s ros-noetic-xgc2-fast-lio2 >/dev/null
dpkg -s ros-noetic-xgc2-swarm-lio2 >/dev/null
dpkg -s ros-noetic-livox-ros-driver >/dev/null
dpkg -s ros-noetic-xgc2-swarm-msgs >/dev/null
dpkg -s ros-noetic-xgc2-udp-bridge >/dev/null
dpkg -s ros-noetic-xgc2-point-lio >/dev/null
dpkg -s ros-noetic-xgc2-lio-sam >/dev/null
dpkg -s ros-noetic-xgc2-voxel-slam >/dev/null
dpkg -s ros-noetic-xgc2-voxelslam-pointcloud2 >/dev/null

test "$(rospack find fast_lio)" = "/opt/ros/${ROS_DISTRO}/share/fast_lio"
test "$(rospack find swarm_lio)" = "/opt/ros/${ROS_DISTRO}/share/swarm_lio"
test "$(rospack find livox_ros_driver)" = "/opt/ros/${ROS_DISTRO}/share/livox_ros_driver"
test "$(rospack find swarm_msgs)" = "/opt/ros/${ROS_DISTRO}/share/swarm_msgs"
test "$(rospack find udp_bridge)" = "/opt/ros/${ROS_DISTRO}/share/udp_bridge"
test "$(rospack find point_lio)" = "/opt/ros/${ROS_DISTRO}/share/point_lio"
test "$(rospack find lio_sam)" = "/opt/ros/${ROS_DISTRO}/share/lio_sam"
test "$(rospack find voxel_slam)" = "/opt/ros/${ROS_DISTRO}/share/voxel_slam"
test "$(rospack find voxelslam_pointcloud2)" = "/opt/ros/${ROS_DISTRO}/share/voxelslam_pointcloud2"

test -x "/opt/ros/${ROS_DISTRO}/lib/fast_lio/fastlio_mapping"
test -x "/opt/ros/${ROS_DISTRO}/lib/swarm_lio/swarm_lio"
test -x "/opt/ros/${ROS_DISTRO}/lib/livox_ros_driver/livox_ros_driver_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/udp_bridge/udp_online"
test -x "/opt/ros/${ROS_DISTRO}/lib/point_lio/pointlio_mapping"
test -x "/opt/ros/${ROS_DISTRO}/lib/lio_sam/lio_sam_imageProjection"
test -x "/opt/ros/${ROS_DISTRO}/lib/lio_sam/lio_sam_featureExtraction"
test -x "/opt/ros/${ROS_DISTRO}/lib/lio_sam/lio_sam_mapOptmization"
test -x "/opt/ros/${ROS_DISTRO}/lib/lio_sam/lio_sam_imuPreintegration"
test -x "/opt/ros/${ROS_DISTRO}/lib/voxel_slam/voxelslam"

test -f "/opt/ros/${ROS_DISTRO}/share/point_lio/config/avia.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/lio_sam/config/params.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/voxel_slam/config/velodyne.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/voxelslam_pointcloud2/plugin_description.xml"
test -f "/opt/ros/${ROS_DISTRO}/lib/libvoxelslam_pointcloud2.so" || test -f "/opt/ros/${ROS_DISTRO}/lib/voxelslam_pointcloud2/libvoxelslam_pointcloud2.so"

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find \
  "/opt/ros/${ROS_DISTRO}/lib/fast_lio" \
  "/opt/ros/${ROS_DISTRO}/lib/swarm_lio" \
  "/opt/ros/${ROS_DISTRO}/lib/livox_ros_driver" \
  "/opt/ros/${ROS_DISTRO}/lib/udp_bridge" \
  "/opt/ros/${ROS_DISTRO}/lib/point_lio" \
  "/opt/ros/${ROS_DISTRO}/lib/lio_sam" \
  "/opt/ros/${ROS_DISTRO}/lib/voxel_slam" \
  "/opt/ros/${ROS_DISTRO}/lib/voxelslam_pointcloud2" \
  -maxdepth 2 -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
