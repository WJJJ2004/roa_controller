#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

POLICY_DIR="${REPO_ROOT}/lib/roa_policy_driver"
POLICY_BUILD="${POLICY_DIR}/build"

echo "[1/4] Build roa_policy_driver"

rm -rf "${POLICY_BUILD}"
cmake -S "${POLICY_DIR}" \
      -B "${POLICY_BUILD}" \
      -DROA_BUILD_TESTS=ON \
      -DCMAKE_BUILD_TYPE=Release

cmake --build "${POLICY_BUILD}" -j"$(nproc)"

echo "[2/4] Test roa_policy_driver"
ctest --test-dir "${POLICY_BUILD}" --output-on-failure

echo "[3/4] Install roa_policy_driver"
sudo cmake --install "${POLICY_BUILD}"
sudo ldconfig

echo "[4/4] Build ROS2 packages (including packet_manager)"

cd "${REPO_ROOT}"

colcon build \
  --base-paths msgs ros2_pkg lib/roa_packet_manager \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build completed successfully."
echo "Run: source install/setup.bash"