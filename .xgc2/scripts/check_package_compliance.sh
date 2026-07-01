#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

required_files=(
  ".xgc2/product.yml"
  ".xgc2/scripts/build_debs_in_docker.sh"
  ".xgc2/scripts/check_installed_packages.sh"
  ".xgc2/scripts/check_package_compliance.sh"
  ".xgc2/scripts/check_ros_packages.sh"
  ".xgc2/scripts/package_debs.sh"
  ".xgc2/scripts/publish_apt_repo.sh"
  ".github/workflows/ci.yml"
  ".github/workflows/release.yml"
  "README.md"
  "fast_lio/CMakeLists.txt"
  "fast_lio/package.xml"
  "swarm_lio2/swarm_msgs/CMakeLists.txt"
  "swarm_lio2/swarm_msgs/package.xml"
  "swarm_lio2/udp_bridge/CMakeLists.txt"
  "swarm_lio2/udp_bridge/package.xml"
  "swarm_lio2/swarm_lio/CMakeLists.txt"
  "swarm_lio2/swarm_lio/package.xml"
  "point_lio/CMakeLists.txt"
  "point_lio/package.xml"
  "lio_sam/CMakeLists.txt"
  "lio_sam/package.xml"
  "voxel_slam/voxel_slam/CMakeLists.txt"
  "voxel_slam/voxel_slam/package.xml"
  "voxel_slam/voxelslam_pointcloud2/CMakeLists.txt"
  "voxel_slam/voxelslam_pointcloud2/package.xml"
)

for file in "${required_files[@]}"; do
  test -f "${REPO_ROOT}/${file}" || {
    echo "missing required file: ${file}" >&2
    exit 1
  }
done

grep -q "id: xgc2-slam" "${REPO_ROOT}/.xgc2/product.yml"
grep -q "ros-noetic-xgc2-fast-lio2" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-swarm-lio2" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-livox-ros-driver" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-point-lio" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-lio-sam" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-voxel-slam" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-voxelslam-pointcloud2" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-slam" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "prune_installed_package_payload" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "message_headers_for_package" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "require_ros_package_payload" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "PACKAGE_VERSION:-1.1.0-1" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "workflow_dispatch:" "${REPO_ROOT}/.github/workflows/release.yml"
grep -q "publish_apt:" "${REPO_ROOT}/.github/workflows/release.yml"
grep -q "publish_apt_repo.sh --deb-dir debs" "${REPO_ROOT}/.github/workflows/release.yml"

echo "Package compliance check passed"
