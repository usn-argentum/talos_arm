#!/bin/bash
# install_service.sh
# Run once on the Jetson after building this package.
# Installs the systemd service and enables auto-start.
#
# This intentionally does NOT start arm_bench_feedback_node or rviz2 -
# see launch/bringup.launch.py for why. Once the real Teensy micro-ROS
# client is running and publishing /arm_joint_states, this service is
# all that's needed on the Jetson side.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/2] Installing systemd service..."
sudo cp "$SCRIPT_DIR/talos_arm.service" /etc/systemd/system/talos_arm.service
sudo systemctl daemon-reload
sudo systemctl enable talos_arm.service
sudo systemctl start talos_arm.service

echo "[2/2] Done."
echo ""
echo "Commands:"
echo "  sudo systemctl status talos_arm    # status"
echo "  sudo systemctl restart talos_arm   # restart"
echo "  journalctl -u talos_arm -f         # live logs"
