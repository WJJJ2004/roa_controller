#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"

POLICY_DIR="${REPO_ROOT}/lib/roa_policy_driver"
POLICY_BUILD="${POLICY_DIR}/build"
POLICY_INSTALL="${WS_ROOT}/install/roa_policy_driver"

echo "[1/4] Build roa_policy_driver"

rm -rf "${POLICY_BUILD}" "${POLICY_INSTALL}"

cmake -S "${POLICY_DIR}" \
      -B "${POLICY_BUILD}" \
      -DROA_BUILD_TESTS=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${POLICY_INSTALL}"

cmake --build "${POLICY_BUILD}" -j"$(nproc)"

echo "[2/4] Test roa_policy_driver"
ctest --test-dir "${POLICY_BUILD}" --output-on-failure

echo "[3/4] Install roa_policy_driver to workspace"
cmake --install "${POLICY_BUILD}"

export CMAKE_PREFIX_PATH="${POLICY_INSTALL}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${POLICY_INSTALL}/lib:${LD_LIBRARY_PATH:-}"

echo "[4/4] Build ROS2 packages"

cd "${REPO_ROOT}"

colcon build \
  --build-base "${WS_ROOT}/build" \
  --install-base "${WS_ROOT}/install" \
  --base-paths msgs ros2_pkg lib/roa_packet_manager \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"

echo "Build completed successfully."
echo "Run: source ${WS_ROOT}/install/setup.bash"