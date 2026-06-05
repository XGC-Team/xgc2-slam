#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${ROS_PACKAGE_CHECK_WORK_DIR:-${REPO_ROOT}/.work/package-tests}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

rm -rf "${WORK_DIR}/src" "${WORK_DIR}/build" "${WORK_DIR}/devel"
mkdir -p "${WORK_DIR}/src"
rsync -a --delete "${REPO_ROOT}/fast_lio/" "${WORK_DIR}/src/fast_lio/"
rsync -a --delete "${REPO_ROOT}/swarm_lio2/livox_ros_driver_mars/" "${WORK_DIR}/src/livox_ros_driver_mars/"
rsync -a --delete "${REPO_ROOT}/swarm_lio2/swarm_msgs/" "${WORK_DIR}/src/swarm_msgs/"
rsync -a --delete "${REPO_ROOT}/swarm_lio2/udp_bridge/" "${WORK_DIR}/src/udp_bridge/"
rsync -a --delete "${REPO_ROOT}/swarm_lio2/swarm_lio/" "${WORK_DIR}/src/swarm_lio/"
rsync -a --delete "${REPO_ROOT}/point_lio/" "${WORK_DIR}/src/point_lio/"
rsync -a --delete "${REPO_ROOT}/lio_sam/" "${WORK_DIR}/src/lio_sam/"
rsync -a --delete "${REPO_ROOT}/voxel_slam/voxel_slam/" "${WORK_DIR}/src/voxel_slam/"
rsync -a --delete "${REPO_ROOT}/voxel_slam/voxelslam_pointcloud2/" "${WORK_DIR}/src/voxelslam_pointcloud2/"

cd "${WORK_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "${WORK_DIR}/devel/setup.bash"
test "$(rospack find fast_lio)" = "${WORK_DIR}/src/fast_lio"
test "$(rospack find swarm_lio)" = "${WORK_DIR}/src/swarm_lio"
test "$(rospack find livox_ros_driver)" = "${WORK_DIR}/src/livox_ros_driver_mars"
test "$(rospack find point_lio)" = "${WORK_DIR}/src/point_lio"
test "$(rospack find lio_sam)" = "${WORK_DIR}/src/lio_sam"
test "$(rospack find voxel_slam)" = "${WORK_DIR}/src/voxel_slam"
test "$(rospack find voxelslam_pointcloud2)" = "${WORK_DIR}/src/voxelslam_pointcloud2"

echo "ROS package check passed"
