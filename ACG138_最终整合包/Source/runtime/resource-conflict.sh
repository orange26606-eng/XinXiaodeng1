#!/bin/sh
# 只读检查资源占用，不结束或接管其他版本的进程。
set -eu
command -v ss >/dev/null 2>&1 || exit 0
busy=$(ss -lntp 2>/dev/null | awk '$4 ~ /:8080$/ {print}')
[ -n "$busy" ] || exit 0
printf '启动被阻止：手机服务端口8080已由其他进程占用。\n%s\n' "$busy" >&2
pid=$(printf '%s\n' "$busy" | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' | head -n 1)
case "$pid" in ''|*[!0-9]*) exit 1;;esac
exe=$(readlink "${ROBOT_PROC_DIR:-/proc}/$pid/exe" 2>/dev/null || :)
if [ -n "$exe" ];then
 printf '占用程序：%s\n' "$exe" >&2
 base=${exe%/*}
 if [ -f "$base/begin" ];then printf '请先在原版本目录手动停止：\ncd "%s"\nsh begin stop\n' "$base" >&2;fi
fi
echo '不会自动杀进程，也不通过改端口并行抢占传感器。' >&2
exit 1
