#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/3] Build roa_policy_driver"
cmake -S "$REPO_ROOT/lib/roa_policy_driver" \
      -B "$REPO_ROOT/build/roa_policy_driver" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$REPO_ROOT/install/roa_policy_driver"

cmake --build "$REPO_ROOT/build/roa_policy_driver" -j
cmake --install "$REPO_ROOT/build/roa_policy_driver"

echo "[2/3] Source policy driver"
export CMAKE_PREFIX_PATH="$REPO_ROOT/install/roa_policy_driver:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$REPO_ROOT/install/roa_policy_driver/lib:$LD_LIBRARY_PATH"

echo "[3/3] Build ROS2 packages"
colcon build \
  --base-paths "$REPO_ROOT/msgs" "$REPO_ROOT/ros2_pkg" \
  --symlink-install \
  --cmake-args -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
