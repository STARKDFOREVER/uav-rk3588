// YAML 配置加载接口，职责：读取单个 YAML 文件并填充 AppConfig
#ifndef _ONBOARD_CONFIG_FILE_H_
#define _ONBOARD_CONFIG_FILE_H_

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "onboard/ego_planner_adapter.h"
#include "onboard/offboard_pilot.h"
#include "onboard/mission_input.h"
#include "onboard/livox_source.h"

namespace ego_onboard
{
  struct AppConfig
  {
    OffboardPilot::Config pilot;          // 飞控时序/安全阈值
    MissionInputConfig mission_input;     // KML 等待与解析
    LivoxSource::Config lidar;            // 雷达帧组装与过滤
    EgoPlannerAdapterConfig adapter;      // 建图与局部规划
    double vehicle_radius = 0.85;         // 机体包络半径
    double min_obstacle_gap = 1.5;       // 规划器要求的障碍物最小间隙
  };

  bool loadConfigFile(const std::string &path, AppConfig &out, std::string &err);
} // namespace ego_onboard

#endif
