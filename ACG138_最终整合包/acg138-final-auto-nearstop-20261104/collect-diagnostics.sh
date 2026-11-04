#!/bin/sh
# 复制诊断数据，不发送STOP、不接管会话、不修改网络。
set -eu
APP=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
: "${ROBOT_RUN_DIR:=/run/acg138-final-auto-nearstop-app}"
: "${ACG_PHONE_DIR:=/run/acg138-final-auto-nearstop-sensors}"
destination=$(mktemp -d /tmp/acg138-diagnostics.XXXXXX)
mkdir -p "$destination/logs" "$destination/state"
for module in sensor vision objects map phone;do
 for suffix in '' .1 .2;do
  source_file="$ROBOT_RUN_DIR/$module.log$suffix"
  [ ! -f "$source_file" ] || cp "$source_file" "$destination/logs/$module.log$suffix"
 done
done
for name in mpu9250-status.json mode-state.json objects-state.json objects-detections.json voice-state.json objects-map.json objects-association.json diagnostic-policy.json;do
 [ ! -f "$ACG_PHONE_DIR/$name" ] || cp "$ACG_PHONE_DIR/$name" "$destination/state/$name"
done
for name in diagnostic-policy.json R81_YOLO_FPGA_IMU_PROVENANCE.json FPGA_SHA256.txt;do
 [ ! -f "$APP/$name" ] || cp "$APP/$name" "$destination/$name"
done
map_session=$(sed -n 's/^Logs: //p' "$ROBOT_RUN_DIR/map.log" 2>/dev/null | tail -n 1)
case "$map_session" in
 /tmp/acg138-final-auto-nearstop.*)
  case "$map_session" in *'/../'*|*'/./'*) :;;*)
   for name in navigation.log navigation.log.1 navigation.log.2 lidar.log lidar.log.1 lidar.log.2;do
    [ ! -f "$map_session/$name" ] || cp "$map_session/$name" "$destination/logs/$name"
   done;;
  esac;;
esac
{
 date
 uname -a
 printf '\n网口与路由：\n'
 ip -4 addr
 ip -4 route
 printf '\n进程与资源：\n'
 ps
 df -h
 printf '\nUSB串口：\n'
 ls -l /dev/ttyUSB* 2>/dev/null || :
} > "$destination/system.txt" 2>&1
archive="$destination.tar.gz"
tar -czf "$archive" -C "$destination" .
printf '诊断已复制，当前程序继续运行：%s\n' "$archive"
