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
rsync -a --delete "${REPO_ROOT}/faster_lio/" "${WORK_DIR}/src/faster_lio/"

cd "${WORK_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "${WORK_DIR}/devel/setup.bash"
test "$(rospack find faster_lio)" = "${WORK_DIR}/src/faster_lio"

echo "ROS package check passed"
