#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
VERSION="${PACKAGE_VERSION:-1.0.0-1}"
PACKAGE_GROUP="${PACKAGE_GROUP:-all}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package-group)
      PACKAGE_GROUP="$2"
      shift 2
      ;;
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
Description: ${description}
EOF
  printf '%s package\n' "${package}" > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

copy_ros_package_paths() {
  local ros_pkg="$1"
  local dst_root="$2"

  copy_path "${PREFIX_ROOT}/share/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/${ros_pkg}" "${dst_root}"
}

build_ros_package_deb() {
  local package="$1"
  local ros_pkg="$2"
  local depends="$3"
  local description="$4"

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

livox_pkg="ros-noetic-xgc2-livox-ros-driver"
fast_pkg="ros-noetic-xgc2-fast-lio2"
swarm_msgs_pkg="ros-noetic-xgc2-swarm-msgs"
udp_pkg="ros-noetic-xgc2-udp-bridge"
swarm_pkg="ros-noetic-xgc2-swarm-lio2"
meta_pkg="ros-noetic-xgc2-slam"

ros_base_depends="ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-std-msgs, ros-noetic-sensor-msgs, ros-noetic-geometry-msgs, ros-noetic-nav-msgs"
lio_depends="${ros_base_depends}, ros-noetic-tf, ros-noetic-pcl-ros, ros-noetic-eigen-conversions, libeigen3-dev, python3, python3-dev"

build_fast_lio2_debs() {
  build_ros_package_deb \
    "${livox_pkg}" \
    "livox_ros_driver" \
    "ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-std-msgs, ros-noetic-sensor-msgs, ros-noetic-message-runtime, ros-noetic-rosbag, ros-noetic-pcl-ros, libapr1" \
    "XGC2 Livox ROS driver support package"

  build_ros_package_deb \
    "${fast_pkg}" \
    "fast_lio" \
    "${lio_depends}, ros-noetic-message-runtime, ${livox_pkg} (= ${VERSION})" \
    "XGC2 FAST-LIO2 LiDAR-inertial odometry package"
}

build_swarm_lio2_debs() {
  build_ros_package_deb \
    "${swarm_msgs_pkg}" \
    "swarm_msgs" \
    "ros-noetic-message-runtime, ros-noetic-std-msgs, ros-noetic-sensor-msgs, ros-noetic-geometry-msgs, ros-noetic-nav-msgs" \
    "XGC2 Swarm-LIO2 message package"

  build_ros_package_deb \
    "${udp_pkg}" \
    "udp_bridge" \
    "${ros_base_depends}, ros-noetic-mavros-msgs, ros-noetic-roslib, ros-noetic-rosbag, ros-noetic-rosfmt, ${swarm_msgs_pkg} (= ${VERSION})" \
    "XGC2 Swarm-LIO2 UDP bridge package"

  build_ros_package_deb \
    "${swarm_pkg}" \
    "swarm_lio" \
    "${lio_depends}, ros-noetic-message-runtime, ros-noetic-gtsam, libtbb2, ${livox_pkg} (= ${VERSION}), ${swarm_msgs_pkg} (= ${VERSION}), ${udp_pkg} (= ${VERSION})" \
    "XGC2 Swarm-LIO2 cooperative LiDAR-inertial odometry package"

  meta_root="${BUILD_DIR}/${meta_pkg}"
  rm -rf "${meta_root}"
  mkdir -p "${meta_root}"
  write_control \
    "${meta_root}" \
    "${meta_pkg}" \
    "${fast_pkg} (= ${VERSION}), ${swarm_pkg} (= ${VERSION})" \
    "XGC2 ROS1 SLAM package set"
  fakeroot dpkg-deb --build "${meta_root}" "${OUTPUT_DIR}/${meta_pkg}_${VERSION}_${ARCH}.deb" >/dev/null
}

case "${PACKAGE_GROUP}" in
  all)
    build_fast_lio2_debs
    build_swarm_lio2_debs
    ;;
  fast-lio2)
    build_fast_lio2_debs
    ;;
  swarm-lio2)
    build_swarm_lio2_debs
    ;;
  *)
    echo "unknown package group: ${PACKAGE_GROUP}" >&2
    exit 1
    ;;
esac

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
