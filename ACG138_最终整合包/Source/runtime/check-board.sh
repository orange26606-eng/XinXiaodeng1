#!/bin/sh
# 只读检查，不模拟按键、不发送电机命令、不采集相机。
set -eu
: "${ACG_PHONE_DIR:=/run/acg138-final-auto-nearstop-sensors}"
echo '--- Kernel and architecture ---'
uname -a
echo '--- Internal FPGA Ethernet (expected 192.168.0.3) ---'
ip -4 addr show
echo '--- Wayland sockets ---'
ls -l /run/wayland-* /run/user/0/wayland-* 2>/dev/null || :
echo '--- Export snapshot ---'
ls -l "$ACG_PHONE_DIR/frames.shm" "$ACG_PHONE_DIR/mpu9250.timed.snapshot" "$ACG_PHONE_DIR/mpu9250.history" 2>/dev/null || :
echo '--- Internal FPGA route (read-only) ---'
ip -4 route get 192.168.0.2 2>/dev/null || :
echo '--- Running camera/phone processes ---'
ps | grep -E 'camera-imu|camera-producer|acg138-phone' | grep -v grep || :
echo '--- Binaries ---'
SELF=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
ls -l "$SELF/acg138-phone-server" "$SELF/camera-imu-panel-phone"
