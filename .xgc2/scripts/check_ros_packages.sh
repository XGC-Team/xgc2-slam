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

cd "${WORK_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "${WORK_DIR}/devel/setup.bash"
test "$(rospack find fast_lio)" = "${WORK_DIR}/src/fast_lio"
test "$(rospack find swarm_lio)" = "${WORK_DIR}/src/swarm_lio"
test "$(rospack find livox_ros_driver)" = "${WORK_DIR}/src/livox_ros_driver_mars"

echo "ROS package check passed"
