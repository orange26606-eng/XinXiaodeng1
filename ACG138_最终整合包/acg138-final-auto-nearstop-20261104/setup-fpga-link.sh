#!/bin/sh
# 幂等配置板内FPGA链路，不修改Wi-Fi网口。
set -eu
iface=${1:-${ROBOT_FPGA_IFACE:-eth0}}
sys=${ROBOT_SYS_DIR:-/sys}
case "$iface" in ''|*[!A-Za-z0-9_.-]*|lo|wl*) echo '拒绝配置无线、回环或无效网口。' >&2;exit 2;;esac
[ -d "$sys/class/net/$iface" ] || { echo "板内网口不存在：$iface" >&2;exit 2; }
[ ! -d "$sys/class/net/$iface/wireless" ] || { echo '拒绝将FPGA地址写入无线网口。' >&2;exit 2; }
owner=$(ip -o -4 addr show | awk '$3=="inet" {split($4,a,"/");if(a[1]=="192.168.0.3"&&!seen[$2]++)print $2}')
[ -z "$owner" ] || [ "$owner" = "$iface" ] || { echo "192.168.0.3 已属于 $owner；不会修改其他网口。" >&2;exit 2; }
routes=$(ip -4 route show 192.168.0.0/24)
if [ -n "$routes" ] && ! printf '%s\n' "$routes" | awk -v dev="$iface" '{for(i=1;i<NF;i++)if($i=="dev"&&$(i+1)!=dev)exit 1}';then
 echo 'FPGA子网已路由到其他网口；拒绝覆盖。' >&2;exit 2
fi
# 相机、手动控制、IMU和底盘反馈共用此链路。
# 在上述服务启动前一次性配置巨帧接收，运行中不重置链路。
mtu=$(cat "$sys/class/net/$iface/mtu")
case "$mtu" in ''|*[!0-9]*) echo '无法读取FPGA网口MTU。' >&2;exit 2;;esac
if [ "$mtu" -ne 4000 ];then
 command -v ss >/dev/null 2>&1 || { echo '缺少ss，无法确认共享网口是否空闲；不修改MTU。' >&2;exit 2; }
 if ss -lntu 2>/dev/null | awk '{n=$5;sub(/^.*:/,"",n);if(n=="8080"||n=="5016")found=1} END{exit !found}';then
  echo '共享控制/传感器服务仍在运行；先停止原会话，再配置MTU4000。' >&2;exit 2
 fi
 ip link set dev "$iface" mtu 4000
fi
if [ "$iface" = eth0 ];then
 ip link set dev eth0 up
 ip -4 addr replace 192.168.0.3/24 dev eth0
else
 ip link set dev "$iface" up
 ip -4 addr replace 192.168.0.3/24 dev "$iface"
fi
route=$(ip -4 route get 192.168.0.2)
case " $route " in *" dev $iface "*) :;;*) echo "FPGA路由网口不匹配：$route" >&2;exit 2;;esac
case " $route " in *" src 192.168.0.3 "*) :;;*) echo "FPGA路由源地址不匹配：$route" >&2;exit 2;;esac
printf 'FPGA板内网口已就绪：%s 192.168.0.3/24 MTU4000\n' "$iface"
