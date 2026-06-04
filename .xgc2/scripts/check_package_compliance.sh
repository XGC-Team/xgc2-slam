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
  ".github/workflows/build-debs.yml"
  "README.md"
  "fast_lio/CMakeLists.txt"
  "fast_lio/package.xml"
  "swarm_lio2/livox_ros_driver_mars/CMakeLists.txt"
  "swarm_lio2/livox_ros_driver_mars/package.xml"
  "swarm_lio2/swarm_msgs/CMakeLists.txt"
  "swarm_lio2/swarm_msgs/package.xml"
  "swarm_lio2/udp_bridge/CMakeLists.txt"
  "swarm_lio2/udp_bridge/package.xml"
  "swarm_lio2/swarm_lio/CMakeLists.txt"
  "swarm_lio2/swarm_lio/package.xml"
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
grep -q "ros-noetic-xgc2-slam" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"

echo "Package compliance check passed"
