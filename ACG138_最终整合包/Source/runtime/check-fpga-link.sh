#!/bin/sh
# 相机重试前只读检查前置条件，不修改IP或链路状态。
set -eu
iface=${1:-${PHONE_FPGA_IFACE:-eth0}}
sys=${ROBOT_SYS_DIR:-/sys}
case "$iface" in ''|*[!A-Za-z0-9_.-]*|lo|wl*) echo '无效FPGA网口。' >&2;exit 2;;esac
[ -r "$sys/class/net/$iface/mtu" ] && [ ! -d "$sys/class/net/$iface/wireless" ] || exit 2
mtu=$(cat "$sys/class/net/$iface/mtu")
case "$mtu" in ''|*[!0-9]*) exit 2;;esac
[ "$mtu" -ge 4000 ] || { echo 'CAMERA_WAIT_LINK: MTU不足4000；在启动整车前运行setup-fpga-link.sh，不重置正在运行的共享网口。' >&2;exit 2; }
route=$(ip -4 route get 192.168.0.2)
case " $route " in *" dev $iface "*) :;;*) echo 'CAMERA_WAIT_LINK: FPGA路由不匹配。' >&2;exit 2;;esac
case " $route " in *" src 192.168.0.3 "*) :;;*) echo 'CAMERA_WAIT_LINK: FPGA源地址不匹配。' >&2;exit 2;;esac
