#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-slam >/dev/null
dpkg -s ros-noetic-xgc2-fast-lio2 >/dev/null
dpkg -s ros-noetic-xgc2-swarm-lio2 >/dev/null
dpkg -s ros-noetic-xgc2-livox-ros-driver >/dev/null
dpkg -s ros-noetic-xgc2-swarm-msgs >/dev/null
dpkg -s ros-noetic-xgc2-udp-bridge >/dev/null

test "$(rospack find fast_lio)" = "/opt/ros/${ROS_DISTRO}/share/fast_lio"
test "$(rospack find swarm_lio)" = "/opt/ros/${ROS_DISTRO}/share/swarm_lio"
test "$(rospack find livox_ros_driver)" = "/opt/ros/${ROS_DISTRO}/share/livox_ros_driver"
test "$(rospack find swarm_msgs)" = "/opt/ros/${ROS_DISTRO}/share/swarm_msgs"
test "$(rospack find udp_bridge)" = "/opt/ros/${ROS_DISTRO}/share/udp_bridge"

test -x "/opt/ros/${ROS_DISTRO}/lib/fast_lio/fastlio_mapping"
test -x "/opt/ros/${ROS_DISTRO}/lib/swarm_lio/swarm_lio"
test -x "/opt/ros/${ROS_DISTRO}/lib/livox_ros_driver/livox_ros_driver_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/udp_bridge/udp_online"

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/fast_lio" "/opt/ros/${ROS_DISTRO}/lib/swarm_lio" "/opt/ros/${ROS_DISTRO}/lib/livox_ros_driver" "/opt/ros/${ROS_DISTRO}/lib/udp_bridge" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
