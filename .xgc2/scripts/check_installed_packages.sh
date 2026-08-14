#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-slam >/dev/null
dpkg -s ros-noetic-xgc2-faster-lio >/dev/null

test "$(rospack find faster_lio)" = "/opt/ros/${ROS_DISTRO}/share/faster_lio"
test -x "/opt/ros/${ROS_DISTRO}/lib/faster_lio/run_mapping_online"
test -f "/opt/ros/${ROS_DISTRO}/share/faster_lio/config/scout_helios16.yaml"

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
  "/opt/ros/${ROS_DISTRO}/lib/faster_lio" \
  -maxdepth 2 -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
