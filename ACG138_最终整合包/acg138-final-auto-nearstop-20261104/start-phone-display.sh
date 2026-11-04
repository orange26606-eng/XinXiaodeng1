#!/bin/sh
# 通过临时目录启动，保留原应用文件和二进制程序。
set -eu
SELF=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
 echo "Usage: $0 /absolute/path/to/existing/rk3588/app [frames]" >&2;exit 2
fi
BASE=$(CDPATH= cd -P -- "$1" && pwd)
for item in run-camera-imu-fast run-camera-imu camera-producer camera-imu-panel;do
 [ -x "$BASE/$item" ] || { echo "Missing original app executable: $BASE/$item" >&2;exit 2; }
done
[ -x "$SELF/camera-imu-panel-phone" ] || { echo "chmod +x $SELF/camera-imu-panel-phone" >&2;exit 2; }
export ACG_PHONE_DIR=${ACG_PHONE_DIR:-/run/acg138-final-auto-nearstop-sensors}
export ACG_PHONE_VIDEO_FIT=${ACG_PHONE_VIDEO_FIT:-yolo640}
mkdir -p "$ACG_PHONE_DIR"
OVERLAY=$(mktemp -d "$ACG_PHONE_DIR/app.XXXXXX")
for source in "$BASE"/*;do [ -e "$source" ] || continue;ln -s "$source" "$OVERLAY/$(basename "$source")";done
ln -sfn "$SELF/camera-imu-panel-phone" "$OVERLAY/camera-imu-panel"
echo "Phone export: $ACG_PHONE_DIR; overlay: $OVERLAY"
echo "Video fit: $ACG_PHONE_VIDEO_FIT (aspect preserved; YOLO status over image)"
echo 'Keep this interactive terminal open; original DSI0/YOLO startup follows.'
if [ "$#" = 2 ];then exec "$OVERLAY/run-camera-imu-fast" "$2";fi
exec "$OVERLAY/run-camera-imu-fast"
