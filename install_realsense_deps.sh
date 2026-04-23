#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="${ROOT_DIR}/.deps"
SRC_DIR="${DEPS_DIR}/librealsense-src"
BUILD_DIR="${DEPS_DIR}/librealsense-build"
INSTALL_DIR="${DEPS_DIR}/realsense2"

LIBREALSENSE_REPO="${LIBREALSENSE_REPO:-https://github.com/IntelRealSense/librealsense.git}"
LIBREALSENSE_REF="${LIBREALSENSE_REF:-v2.57.7}"
JOBS="${JOBS:-$(nproc)}"

mkdir -p "${DEPS_DIR}"

if [ ! -d "${SRC_DIR}/.git" ]; then
  echo "Cloning librealsense (${LIBREALSENSE_REF}) into ${SRC_DIR}"
  if ! git clone --depth 1 --branch "${LIBREALSENSE_REF}" "${LIBREALSENSE_REPO}" "${SRC_DIR}"; then
    echo "Branch/tag ${LIBREALSENSE_REF} was not available via shallow clone; retrying full checkout."
    git clone "${LIBREALSENSE_REPO}" "${SRC_DIR}"
    git -C "${SRC_DIR}" checkout "${LIBREALSENSE_REF}"
  fi
else
  echo "Using existing librealsense source at ${SRC_DIR}"
  if [ "${UPDATE_DEPS:-0}" = "1" ]; then
    echo "Updating librealsense source to ${LIBREALSENSE_REF}"
    git -C "${SRC_DIR}" fetch --tags origin
    git -C "${SRC_DIR}" checkout "${LIBREALSENSE_REF}"
  fi
fi

echo "Configuring librealsense install prefix: ${INSTALL_DIR}"
cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_SHARED_LIBS=ON \
  -DFORCE_RSUSB_BACKEND=ON

echo "Building and installing librealsense with ${JOBS} job(s)"
cmake --build "${BUILD_DIR}" --target install -- -j"${JOBS}"

echo
echo "librealsense installed under:"
echo "  ${INSTALL_DIR}"
echo
echo "Reconfigure NGD-SLAM with:"
echo "  cmake -S ${ROOT_DIR} -B ${ROOT_DIR}/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=${INSTALL_DIR}"
echo
echo "If D455 USB permission is denied later, install udev rules with:"
echo "  sudo bash ${SRC_DIR}/scripts/setup_udev_rules.sh"
