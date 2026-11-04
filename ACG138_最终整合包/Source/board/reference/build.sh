#!/bin/sh
# 仅在独立Linux工作副本构建，不安装或运行客户端。
set -eu
cd "$(dirname "$0")"
HOST=${BUILDROOT_HOST:?set BUILDROOT_HOST to the matching Buildroot output/host directory}
SYSROOT=${BUILDROOT_SYSROOT:-"$HOST/aarch64-buildroot-linux-gnu/sysroot"}
OUT=${IMU_BUILD_OUT:-output}
CC="$HOST/bin/aarch64-buildroot-linux-gnu-gcc"
SCANNER="$HOST/bin/wayland-scanner"
test -x "$CC" || { echo "Missing matching AArch64 compiler: $CC" >&2; exit 1; }
test -x "$SCANNER" || { echo "Missing Wayland scanner: $SCANNER" >&2; exit 1; }
test -f "$SYSROOT/usr/include/wayland-client.h" || { echo "Missing Wayland headers in sysroot" >&2; exit 1; }
mkdir -p "$OUT"
for protocol in xdg-shell viewporter; do
    xml="$SYSROOT/usr/share/wayland-protocols/stable/$protocol/$protocol.xml"
    test -f "$xml" || { echo "Missing protocol XML: $xml" >&2; exit 1; }
    "$SCANNER" client-header "$xml" "$OUT/$protocol-client-protocol.h"
    "$SCANNER" private-code "$xml" "$OUT/$protocol-protocol.c"
done
xml="$SYSROOT/usr/share/wayland-protocols/unstable/xdg-output/xdg-output-unstable-v1.xml"
test -f "$xml" || { echo "Missing protocol XML: $xml" >&2; exit 1; }
"$SCANNER" client-header "$xml" "$OUT/xdg-output-client-protocol.h"
"$SCANNER" private-code "$xml" "$OUT/xdg-output-protocol.c"
"$CC" --sysroot="$SYSROOT" -std=gnu11 -O2 -Wall -Wextra -Werror -I"$OUT" \
    imu_wayland_panel.c imu_udp.c imu_protocol.c imu_canvas.c imu_iic_control.c imu_control_raw.c \
    imu_terminal_gate.c imu_terminal_flow.c \
    "$OUT/xdg-shell-protocol.c" "$OUT/viewporter-protocol.c" "$OUT/xdg-output-protocol.c" \
    -lwayland-client -lm -o "$OUT/imu-wayland-panel"
"$CC" --sysroot="$SYSROOT" -std=gnu11 -O2 -Wall -Wextra -Werror \
    imu_udp_console.c imu_udp.c imu_protocol.c -lm -o "$OUT/imu-udp-console"
cp imu-angle "$OUT/imu-angle"
chmod 755 "$OUT/imu-angle"
echo "Built $OUT/imu-wayland-panel and $OUT/imu-udp-console and launcher imu-angle (not installed or run)."
