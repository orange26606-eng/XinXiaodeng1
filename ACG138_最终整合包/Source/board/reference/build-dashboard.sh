#!/bin/sh
# 版本1.1.2，2026-10-16，仅构建，不安装或访问设备。
# 1.1.0统一面板构建；1.1.1增加更新率、检测结果输入和PTY桥接。
# 1.1.2将相对航向状态纳入面板构建。
# 1.2.0同时构建NV12上游桥接、显示面板及结果桥接。
set -eu
cd "$(dirname "$0")"
HOST=${BUILDROOT_HOST:?set matching Buildroot host directory}
SYSROOT=${BUILDROOT_SYSROOT:-"$HOST/aarch64-buildroot-linux-gnu/sysroot"}
OUT=${IMU_BUILD_OUT:-output}
CC="$HOST/bin/aarch64-buildroot-linux-gnu-gcc"
SCANNER="$HOST/bin/wayland-scanner"
mkdir -p "$OUT"
for protocol in xdg-shell viewporter; do
    xml="$SYSROOT/usr/share/wayland-protocols/stable/$protocol/$protocol.xml"
    "$SCANNER" client-header "$xml" "$OUT/$protocol-client-protocol.h"
    "$SCANNER" private-code "$xml" "$OUT/$protocol-protocol.c"
done
xml="$SYSROOT/usr/share/wayland-protocols/unstable/xdg-output/xdg-output-unstable-v1.xml"
"$SCANNER" client-header "$xml" "$OUT/xdg-output-client-protocol.h"
"$SCANNER" private-code "$xml" "$OUT/xdg-output-protocol.c"
"$CC" --sysroot="$SYSROOT" -std=gnu11 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -Wno-missing-field-initializers -I"$OUT" \
    ../odometry.c ../odometry_runtime.c ../mpu9250/imu_fusion.c ../mpu9250/mpu_adapter.c camera_imu_panel.c panel_outputs.c panel_output_policy.c ../phone_export.c robot_protocol.c robot_udp.c dashboard_canvas.c dashboard_yolo_fps.c dashboard_heading.c dashboard_video.c dashboard_rate.c dashboard_detections.c imu_udp.c imu_protocol.c \
    imu_iic_control.c imu_control_raw.c imu_auto_start.c \
    "$OUT/xdg-shell-protocol.c" "$OUT/viewporter-protocol.c" "$OUT/xdg-output-protocol.c" \
    -pthread -lwayland-client -lm -o "$OUT/camera-imu-panel"
"$CC" --sysroot="$SYSROOT" -std=gnu11 -O2 -Wall -Wextra -Werror \
    yolo_results_bridge.c dashboard_detections.c -lm -o "$OUT/yolo-results-bridge"
echo "Built camera-imu-panel for aarch64; not run on the board."

"$CC" --sysroot="$SYSROOT" -std=gnu11 -O2 -Wall -Wextra -Werror camera_half_swap.c camera_swap_nv12.c dashboard_video.c -o "$OUT/camera-half-swap"
