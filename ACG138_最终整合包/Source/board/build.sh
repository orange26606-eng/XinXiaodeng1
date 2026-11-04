#!/bin/sh
# 使用匹配的Buildroot SDK构建；主机验证可使用本地Linux gcc。
set -eu
cd "$(dirname "$0")"
OUT=${PHONE_BUILD_OUT:-../build/board}
mkdir -p "$OUT"
if [ -n "${BUILDROOT_HOST:-}" ]; then
 CC="$BUILDROOT_HOST/bin/aarch64-buildroot-linux-gnu-gcc"
 export BUILDROOT_SYSROOT=${BUILDROOT_SYSROOT:-"$BUILDROOT_HOST/aarch64-buildroot-linux-gnu/sysroot"}
else
 CC=${CC:-gcc}
fi
"$CC" -std=gnu11 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -Wno-missing-field-initializers \
 -pthread odometry.c odometry_runtime.c mpu9250/imu_fusion.c mpu9250/mpu_adapter.c phone_server.c video/video_service.c video/h264_encoder.c control_core.c reference/dashboard_video.c reference/robot_protocol.c -lm -ldl -o "$OUT/acg138-phone-server"
"$CC" -std=gnu11 -O2 -pthread odometry_info.c odometry.c odometry_runtime.c mpu9250/imu_fusion.c mpu9250/mpu_adapter.c -lm -o "$OUT/acg138-odometry-info"
if [ -n "${BUILDROOT_HOST:-}" ]; then
 export IMU_BUILD_OUT="$(cd "$OUT" && pwd)/panel"
 (cd reference && sh build-dashboard.sh)
 cp "$IMU_BUILD_OUT/camera-imu-panel" "$OUT/camera-imu-panel-phone"
fi
