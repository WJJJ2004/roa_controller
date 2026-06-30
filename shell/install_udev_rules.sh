#!/usr/bin/env bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMU_PKG_DIR="${ROOT_DIR}/ros2_pkg/imu_pkg"

RULE_SRC="${IMU_PKG_DIR}/udev/tty-usb.rules"
RULE_DST="/etc/udev/rules.d/tty-usb.rules"

echo "[INFO] imu_pkg directory: ${IMU_PKG_DIR}"

if [ ! -f "${RULE_SRC}" ]; then
    echo "[ERROR] udev rule file not found: ${RULE_SRC}"
    exit 1
fi

echo "[INFO] Installing EBIMU udev rule..."
echo "[INFO] ${RULE_SRC} -> ${RULE_DST}"

sudo cp "${RULE_SRC}" "${RULE_DST}"
sudo chmod 644 "${RULE_DST}"

echo "[INFO] Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[INFO] Done."
echo
echo "Check symbolic link:"
echo "  ls -l /dev/ttyUSB-EBIMU"
echo
echo "If the device is already connected, unplug and reconnect it."