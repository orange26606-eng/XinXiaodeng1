#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$root"
sha256sum -c SHA256SUMS.txt
for file in begin begin-phone robotctl robot-pty s5-manager-event s5-diagnose s6-diagnose setup-fpga-link.sh check-fpga-link.sh resource-conflict.sh run-mpu-sensors nav/run-live-nav nav/live-map-nav-panel lab-mpu9250-service lab-mpu9250-info acg138-apriltag acg138-objects acg138-phone-server camera-imu-panel-phone startPhoneV7 start-phone-display.sh start-phone-server.sh map/lidar-usb-timed;do chmod u+x "$file";done
echo 'Verified only. No services started, no network changes or motor commands.'
