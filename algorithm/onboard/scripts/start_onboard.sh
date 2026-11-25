#!/usr/bin/env bash
# One process per boot/task. The executable owns WAIT_KML and task validation.
# 用途：systemd 调用入口，创建本次日志目录并启动实机飞行控制。
# 前置条件：.build/onboard/onboard_main 已构建；串口/波特率/配置/KML 可由环境变量覆盖。
set -euo pipefail
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$project"
binary="$project/.build/onboard/onboard_main"
# 未构建时返回 2，避免服务静默失败。
[[ -x "$binary" ]] || { echo 'Build .build/onboard/onboard_main before deployment.' >&2; exit 2; }
# 每次运行使用独立时间戳目录，标准输出同时写入 onboard.log 便于事后排查。
record_root="$project/log"
mkdir -p "$record_root"
record_dir="$record_root/$(date +%Y%m%d_%H%M%S)_$$"
mkdir "$record_dir"
record_args=(--record-dir "$record_dir")
echo "Recording diagnostics to $record_dir"
exec > >(tee -a "$record_dir/onboard.log") 2>&1
# 默认串口用 by-id 路径避免设备名漂移；雷达 IP 固定为 192.168.1.173。
exec stdbuf -oL -eL "$binary" \
  --serial "${UAV_SERIAL:-/dev/serial/by-id/usb-ACFLy_ACFLy_Device_123456-if00}" \
  --baud "${UAV_BAUD:-921600}" \
  --config "${UAV_CONFIG:-$project/algorithm/onboard/config/onboard.yaml}" \
  --kml "${UAV_KML:-$project/missions/mission.kml}" \
  --livox-config "$project/algorithm/onboard/deploy/mid360s.json" \
  --lidar-ip 192.168.1.173 \
  --error-dir "$record_dir" --enable-control "${record_args[@]}"
