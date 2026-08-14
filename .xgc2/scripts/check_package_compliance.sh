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
  ".github/workflows/ci.yml"
  ".github/workflows/release.yml"
  "README.md"
  "faster_lio/CMakeLists.txt"
  "faster_lio/package.xml"
)

for file in "${required_files[@]}"; do
  test -f "${REPO_ROOT}/${file}" || {
    echo "missing required file: ${file}" >&2
    exit 1
  }
done

grep -q "id: xgc2-slam" "${REPO_ROOT}/.xgc2/product.yml"
grep -q "ros-noetic-xgc2-faster-lio" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-slam" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "prune_installed_package_payload" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "message_headers_for_package" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "require_ros_package_payload" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -Fq 'PACKAGE_VERSION:-$(product_version)' "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "workflow_dispatch:" "${REPO_ROOT}/.github/workflows/release.yml"

echo "Package compliance check passed"
