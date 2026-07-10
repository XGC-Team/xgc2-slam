#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
PACKAGE_GROUP="${PACKAGE_GROUP:-all}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package-group)
      PACKAGE_GROUP="$2"
      shift 2
      ;;
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  --network host \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e PACKAGE_GROUP="${PACKAGE_GROUP}" \
  -v "${REPO_ROOT}:/workspace/slam:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    apt_update_retry() {
      local attempt
      for attempt in 1 2 3 4; do
        rm -rf /var/lib/apt/lists/*
        if apt-get update; then
          return 0
        fi
        sleep "$((attempt * 10))"
      done
      rm -rf /var/lib/apt/lists/*
      apt-get update
    }

    export DEBIAN_FRONTEND=noninteractive
    apt_update_retry
    apt-get install -y --no-install-recommends ca-certificates curl
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list

      if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
        sed "s#${XGC2_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}#${XGC2_APT_OVERLAY_URL%/}#g" \
          /etc/apt/sources.list.d/xgc2.list \
          > /etc/apt/sources.list.d/00-xgc2-release-train.list
      fi
    apt_update_retry

    apt_packages=(
      build-essential \
      ca-certificates \
      cmake \
      curl \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libapr1-dev \
      libeigen3-dev \
      libtbb-dev \
      python3-dev \
      rsync \
      ros-noetic-eigen-conversions \
      ros-noetic-geometry-msgs \
      ros-noetic-mavros-msgs \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-nav-msgs \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-rosbag \
      ros-noetic-roscpp \
      ros-noetic-rosfmt \
      ros-noetic-roslib \
      ros-noetic-rospack \
      ros-noetic-rospy \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs \
      ros-noetic-tf \
      ros-noetic-visualization-msgs
    )
    case "${PACKAGE_GROUP}" in
      all)
        apt_packages+=(libgoogle-glog-dev libopencv-dev qtbase5-dev ros-noetic-cv-bridge ros-noetic-gtsam ros-noetic-livox-ros-driver ros-noetic-rviz)
        ;;
      fast-lio2)
        apt_packages+=(ros-noetic-livox-ros-driver)
        ;;
      point-lio)
        apt_packages+=(libgoogle-glog-dev ros-noetic-livox-ros-driver)
        ;;
      swarm-lio2)
        apt_packages+=(ros-noetic-gtsam ros-noetic-livox-ros-driver)
        ;;
      lio-sam)
        apt_packages+=(libopencv-dev ros-noetic-cv-bridge ros-noetic-gtsam)
        ;;
      voxel-slam)
        apt_packages+=(qtbase5-dev ros-noetic-gtsam ros-noetic-livox-ros-driver ros-noetic-rviz)
        ;;
    esac
    apt-get install -y --no-install-recommends "${apt_packages[@]}"

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src
    case "${PACKAGE_GROUP}" in
      all)
        rsync -a --delete /workspace/slam/fast_lio/ /workspace/work/src/fast_lio/
        rsync -a --delete /workspace/slam/swarm_lio2/swarm_msgs/ /workspace/work/src/swarm_msgs/
        rsync -a --delete /workspace/slam/swarm_lio2/udp_bridge/ /workspace/work/src/udp_bridge/
        rsync -a --delete /workspace/slam/swarm_lio2/swarm_lio/ /workspace/work/src/swarm_lio/
        rsync -a --delete /workspace/slam/point_lio/ /workspace/work/src/point_lio/
        rsync -a --delete /workspace/slam/lio_sam/ /workspace/work/src/lio_sam/
        rsync -a --delete /workspace/slam/voxel_slam/voxel_slam/ /workspace/work/src/voxel_slam/
        rsync -a --delete /workspace/slam/voxel_slam/voxelslam_pointcloud2/ /workspace/work/src/voxelslam_pointcloud2/
        ;;
      fast-lio2)
        rsync -a --delete /workspace/slam/fast_lio/ /workspace/work/src/fast_lio/
        ;;
      swarm-lio2)
        rsync -a --delete /workspace/slam/swarm_lio2/swarm_msgs/ /workspace/work/src/swarm_msgs/
        rsync -a --delete /workspace/slam/swarm_lio2/udp_bridge/ /workspace/work/src/udp_bridge/
        rsync -a --delete /workspace/slam/swarm_lio2/swarm_lio/ /workspace/work/src/swarm_lio/
        ;;
      point-lio)
        rsync -a --delete /workspace/slam/point_lio/ /workspace/work/src/point_lio/
        ;;
      lio-sam)
        rsync -a --delete /workspace/slam/lio_sam/ /workspace/work/src/lio_sam/
        ;;
      voxel-slam)
        rsync -a --delete /workspace/slam/voxel_slam/voxel_slam/ /workspace/work/src/voxel_slam/
        rsync -a --delete /workspace/slam/voxel_slam/voxelslam_pointcloud2/ /workspace/work/src/voxelslam_pointcloud2/
        ;;
      *)
        echo "unknown package group: ${PACKAGE_GROUP}" >&2
        exit 1
        ;;
    esac

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/slam/.xgc2/scripts/package_debs.sh \
      --package-group "${PACKAGE_GROUP}" \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/*.deb
      /workspace/slam/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
