#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
VERSION="${PACKAGE_VERSION:-1.1.0-1}"
PACKAGE_GROUP="${PACKAGE_GROUP:-all}"
ARCH="$(dpkg --print-architecture)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package-group)
      PACKAGE_GROUP="$2"
      shift 2
      ;;
    --arch)
      ARCH="$2"
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

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "--output-dir is required" >&2
  exit 1
fi

if [[ "${PACKAGE_GROUP}" != "meta" && -z "${INSTALL_ROOT}" ]]; then
  echo "--install-root is required unless --package-group meta is used" >&2
  exit 1
fi

case "${ARCH}" in
  amd64|arm64)
    ;;
  *)
    echo "unsupported architecture: ${ARCH}" >&2
    exit 1
    ;;
esac

PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*_"${ARCH}".deb

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

  case "${ros_pkg}" in
    voxelslam_pointcloud2)
      copy_path "${PREFIX_ROOT}/lib/libvoxelslam_pointcloud2.so" "${dst_root}"
      ;;
  esac
}

require_ros_package_payload() {
  local ros_pkg="$1"
  local pkg_root="$2"

  if [[ ! -d "${pkg_root}${PREFIX}/share/${ros_pkg}" && ! -d "${pkg_root}${PREFIX}/lib/${ros_pkg}" && ! -d "${pkg_root}${PREFIX}/include/${ros_pkg}" ]]; then
    echo "missing installed payload for ROS package ${ros_pkg}; check that the source package has catkin install() rules" >&2
    exit 1
  fi
}

message_headers_for_package() {
  local ros_pkg="$1"

  case "${ros_pkg}" in
    fast_lio)
      printf '%s\n' Pose6D.h
      ;;
    point_lio)
      printf '%s\n' LocalSensorExternalTrigger.h
      ;;
    lio_sam)
      printf '%s\n' cloud_info.h save_map.h
      ;;
    swarm_lio)
      printf '%s\n' Pose6D.h States.h
      ;;
    swarm_msgs)
      printf '%s\n' \
        ConnectedTeammateList.h \
        GlobalExtrinsic.h \
        GlobalExtrinsicStatus.h \
        ObserveTeammate.h \
        QuadStatePub.h \
        SpatialTemporalOffset.h \
        SpatialTemporalOffsetStatus.h \
        TeamStatus.h \
        TeammateInfo.h
      ;;
  esac
}

prune_installed_package_payload() {
  local pkg_root="$1"
  local ros_pkg="$2"
  local share_dir="${pkg_root}${PREFIX}/share/${ros_pkg}"
  local include_dir="${pkg_root}${PREFIX}/include/${ros_pkg}"
  local keep_dir=""
  local header=""
  local asset=""

  if [[ -d "${share_dir}" ]]; then
    rm -rf \
      "${share_dir}/doc" \
      "${share_dir}/docs"

    find "${share_dir}" -type f \( \
      -iname '*.md' -o \
      -iname '*.pdf' \
    \) -delete

    while IFS= read -r -d '' asset; do
      case "${asset}" in
        */icon/*|*/icons/*|*/media/*|*/mesh/*|*/meshes/*|*/model/*|*/models/*|*/texture/*|*/textures/*|*/urdf/*)
          ;;
        *)
          rm -f "${asset}"
          ;;
      esac
    done < <(
      find "${share_dir}" -type f \( \
        -iname '*.bmp' -o \
        -iname '*.gif' -o \
        -iname '*.jpeg' -o \
        -iname '*.jpg' -o \
        -iname '*.png' -o \
        -iname '*.svg' \
      \) -print0
    )

    find "${share_dir}" -depth -type d -empty -delete
  fi

  if [[ -d "${include_dir}" ]]; then
    keep_dir="$(mktemp -d)"
    while IFS= read -r header; do
      if [[ -f "${include_dir}/${header}" ]]; then
        cp -a "${include_dir}/${header}" "${keep_dir}/${header}"
      fi
    done < <(message_headers_for_package "${ros_pkg}")

    rm -rf "${include_dir}"
    if compgen -G "${keep_dir}/*.h" >/dev/null; then
      mkdir -p "${include_dir}"
      cp -a "${keep_dir}/." "${include_dir}/"
    fi
    rm -rf "${keep_dir}"
  fi
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
  require_ros_package_payload "${ros_pkg}" "${pkg_root}"
  prune_installed_package_payload "${pkg_root}" "${ros_pkg}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

livox_dep="ros-noetic-livox-ros-driver (>= 2.6.0-1)"
fast_pkg="ros-noetic-xgc2-fast-lio2"
swarm_msgs_pkg="ros-noetic-xgc2-swarm-msgs"
udp_pkg="ros-noetic-xgc2-udp-bridge"
swarm_pkg="ros-noetic-xgc2-swarm-lio2"
point_lio_pkg="ros-noetic-xgc2-point-lio"
lio_sam_pkg="ros-noetic-xgc2-lio-sam"
voxel_slam_pkg="ros-noetic-xgc2-voxel-slam"
voxelslam_pointcloud2_pkg="ros-noetic-xgc2-voxelslam-pointcloud2"
meta_pkg="ros-noetic-xgc2-slam"

ros_base_depends="ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-std-msgs, ros-noetic-sensor-msgs, ros-noetic-geometry-msgs, ros-noetic-nav-msgs"
lio_depends="${ros_base_depends}, ros-noetic-tf, ros-noetic-pcl-ros, ros-noetic-pcl-conversions, ros-noetic-eigen-conversions, libeigen3-dev, python3, python3-dev"
gtsam_depends="ros-noetic-gtsam, libtbb2"
opencv_depends="ros-noetic-cv-bridge, libopencv-dev"
rviz_plugin_depends="ros-noetic-rviz, libqt5core5a, libqt5gui5, libqt5widgets5"

build_fast_lio2_debs() {
  build_ros_package_deb \
    "${fast_pkg}" \
    "fast_lio" \
    "${lio_depends}, ros-noetic-message-runtime, ${livox_dep}" \
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
    "${lio_depends}, ros-noetic-message-runtime, ros-noetic-gtsam, libtbb2, ${livox_dep}, ${swarm_msgs_pkg} (= ${VERSION}), ${udp_pkg} (= ${VERSION})" \
    "XGC2 Swarm-LIO2 cooperative LiDAR-inertial odometry package"
}

build_point_lio_debs() {
  build_ros_package_deb \
    "${point_lio_pkg}" \
    "point_lio" \
    "${lio_depends}, ros-noetic-message-runtime, libgoogle-glog0v5, ${livox_dep}" \
    "XGC2 Point-LIO LiDAR-inertial odometry package"
}

build_lio_sam_debs() {
  build_ros_package_deb \
    "${lio_sam_pkg}" \
    "lio_sam" \
    "${ros_base_depends}, ros-noetic-visualization-msgs, ros-noetic-tf, ros-noetic-pcl-conversions, ${opencv_depends}, ${gtsam_depends}, ros-noetic-message-runtime" \
    "XGC2 LIO-SAM LiDAR-inertial smoothing and mapping package"
}

build_voxel_slam_debs() {
  build_ros_package_deb \
    "${voxel_slam_pkg}" \
    "voxel_slam" \
    "${lio_depends}, ros-noetic-rosbag, ros-noetic-visualization-msgs, ${gtsam_depends}, ${livox_dep}" \
    "XGC2 Voxel-SLAM LiDAR mapping package"

  build_ros_package_deb \
    "${voxelslam_pointcloud2_pkg}" \
    "voxelslam_pointcloud2" \
    "${rviz_plugin_depends}" \
    "XGC2 Voxel-SLAM RViz point cloud plugin package"
}

build_meta_deb() {
  meta_root="${BUILD_DIR}/${meta_pkg}"
  rm -rf "${meta_root}"
  mkdir -p "${meta_root}"
  write_control \
    "${meta_root}" \
    "${meta_pkg}" \
    "${fast_pkg} (= ${VERSION}), ${swarm_pkg} (= ${VERSION}), ${point_lio_pkg} (= ${VERSION}), ${lio_sam_pkg} (= ${VERSION}), ${voxel_slam_pkg} (= ${VERSION}), ${voxelslam_pointcloud2_pkg} (= ${VERSION})" \
    "XGC2 ROS1 SLAM package set"
  fakeroot dpkg-deb --build "${meta_root}" "${OUTPUT_DIR}/${meta_pkg}_${VERSION}_${ARCH}.deb" >/dev/null
}

case "${PACKAGE_GROUP}" in
  all)
    build_fast_lio2_debs
    build_swarm_lio2_debs
    build_point_lio_debs
    build_lio_sam_debs
    build_voxel_slam_debs
    build_meta_deb
    ;;
  fast-lio2)
    build_fast_lio2_debs
    ;;
  swarm-lio2)
    build_swarm_lio2_debs
    ;;
  point-lio)
    build_point_lio_debs
    ;;
  lio-sam)
    build_lio_sam_debs
    ;;
  voxel-slam)
    build_voxel_slam_debs
    ;;
  meta)
    build_meta_deb
    ;;
  *)
    echo "unknown package group: ${PACKAGE_GROUP}" >&2
    exit 1
    ;;
esac

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
