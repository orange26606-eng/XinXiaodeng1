#!/bin/sh
set -eu
SELF=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
DIR=${ACG_PHONE_DIR:-/run/acg138-final-auto-nearstop-sensors}
set -- "$@"
VIDEO=${PHONE_VIDEO_CODEC:-h264}
case "$VIDEO" in h264|jpeg) ;; *) echo 'PHONE_VIDEO_CODEC must be h264 or jpeg.' >&2;exit 2;; esac
MPP="$SELF/video/librockchip_mpp.so.0"
if [ "$VIDEO" = h264 ]; then
 if [ ! -f "$MPP" ];then echo "PHONE_VIDEO_DEGRADED: missing $MPP; manual API starts without H264." >&2;VIDEO=jpeg;fi
fi
export ACG_PHONE_VIDEO_FIT=${ACG_PHONE_VIDEO_FIT:-yolo640}
mkdir -p "$DIR"
if [ "$#" -gt 1 ];then echo "Usage: $0 [FPGA_INTERNAL_ETH_INTERFACE]" >&2;exit 2;fi
if [ "$#" = 1 ];then
 [ -d "/sys/class/net/$1" ] || { echo "Network interface not found: $1" >&2;exit 2; }
 if [ "$VIDEO" = h264 ];then exec "$SELF/acg138-phone-server" --dir "$DIR" --enable-control --interface "$1" --h264 --mpp-library "$MPP";fi
 exec "$SELF/acg138-phone-server" --dir "$DIR" --enable-control --interface "$1"
fi
echo 'Monitor mode. To enable the new FPGA receiver, pass the internal RGMII interface name.'
if [ "$VIDEO" = h264 ];then exec "$SELF/acg138-phone-server" --dir "$DIR" --h264 --mpp-library "$MPP";fi
exec "$SELF/acg138-phone-server" --dir "$DIR"
