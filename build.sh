#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${JOBS:-4}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
USE_NATIVE_ARCH="${ORB_SLAM3_USE_NATIVE_ARCH:-OFF}"

echo "Configuring and building Thirdparty/DBoW2 ..."
cmake -S "${ROOT_DIR}/Thirdparty/DBoW2" -B "${ROOT_DIR}/Thirdparty/DBoW2/build" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
cmake --build "${ROOT_DIR}/Thirdparty/DBoW2/build" -- -j"${JOBS}"

echo "Configuring and building Thirdparty/g2o ..."
cmake -S "${ROOT_DIR}/Thirdparty/g2o" -B "${ROOT_DIR}/Thirdparty/g2o/build" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
cmake --build "${ROOT_DIR}/Thirdparty/g2o/build" -- -j"${JOBS}"

echo "Configuring and building Thirdparty/Sophus ..."
cmake -S "${ROOT_DIR}/Thirdparty/Sophus" -B "${ROOT_DIR}/Thirdparty/Sophus/build" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
cmake --build "${ROOT_DIR}/Thirdparty/Sophus/build" -- -j"${JOBS}"

echo "Uncompress vocabulary ..."
if [ ! -f "${ROOT_DIR}/Vocabulary/ORBvoc.txt" ]; then
  tar -xf "${ROOT_DIR}/Vocabulary/ORBvoc.txt.tar.gz" -C "${ROOT_DIR}/Vocabulary"
fi

echo "Configuring and building ORB_SLAM3 ..."
CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
  -DORB_SLAM3_USE_NATIVE_ARCH="${USE_NATIVE_ARCH}"
)

if [ -d "${ROOT_DIR}/.deps/realsense2" ]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=${ROOT_DIR}/.deps/realsense2")
fi

cmake -S "${ROOT_DIR}" -B "${ROOT_DIR}/build" "${CMAKE_ARGS[@]}"
cmake --build "${ROOT_DIR}/build" -- -j"${JOBS}"
