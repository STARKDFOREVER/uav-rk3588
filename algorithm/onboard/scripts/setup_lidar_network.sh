#!/usr/bin/env bash
# 用途：为雷达物理网口配置 192.168.1.50/24 与到雷达 192.168.1.173 的主机路由。
set -euo pipefail
# 参数数量不符时返回 2。
if [[ $# -ne 1 ]]; then
  echo "Usage: sudo $0 <physical-lidar-interface>; sets additional 192.168.1.50/24 address, no default route change" >&2
  exit 2
fi
iface=$1
if [[ ! -d /sys/class/net/$iface ]]; then echo "Interface not found: $iface" >&2; exit 2; fi
# 拉起网口并幂等写入地址与主机路由
ip link set dev "$iface" up
ip address replace 192.168.1.50/24 dev "$iface"
ip route replace 192.168.1.173/32 dev "$iface" src 192.168.1.50
# 打印当前地址供核对，并给出手动启动 PTP 的命令。
ip -br address show dev "$iface"
echo "Start PTP: sudo ptp4l -i $iface -f algorithm/onboard/deploy/ptp4l-master.conf -m"
