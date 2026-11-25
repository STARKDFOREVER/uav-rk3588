#!/usr/bin/env bash
# 用途：绕过飞控，仅经以太网与 Livox SDK2 验证雷达原始点云接收。
# 前置条件：雷达网口已配置，PTP 另行管理。
set -euo pipefail
project=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$project"
# 采集时长参数 5..3600 s，默认 60；非法返回 2。
duration=${1:-60}
[[ "$duration" =~ ^[0-9]+$ ]] && ((duration>=5 && duration<=3600)) || { echo 'Duration must be 5..3600 seconds.' >&2; exit 2; }
# 报告目录按上海时区时间戳命名，存放构建与采集日志。
report="algorithm/onboard/test/reports/lidar-direct-$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)"
mkdir -p "$(dirname "$report")"
mkdir "$report"
# Release 构建，构建日志写入报告目录。
cmake -S algorithm/onboard -B .build/onboard -DCMAKE_BUILD_TYPE=Release > "$report/build.log" 2>&1
cmake --build .build/onboard -j4 >> "$report/build.log" 2>&1
echo 'Direct Ethernet/SDK2 capture. No FCU serial, mission upload or motion commands.'
echo 'PTP is managed separately; this command verifies raw sensor-frame reception.'
# 仅用 --lidar-only 采集：不打开串口、不发任务与运动指令，输出同时写入报告。
stdbuf -oL -eL .build/onboard/onboard_main --lidar-only \
  --config algorithm/onboard/config/onboard.yaml \
  --livox-config algorithm/onboard/deploy/mid360s.json --lidar-ip 192.168.1.173 \
  --duration "$duration" --record-dir "$report/capture" 2>&1 | tee "$report/observe.log"
echo "Capture saved: $project/$report"
