// 任务输入（本地 KML）接口。
// 职责：等待 KML 落盘 -> 读取 -> 严格解析第一条 LineString -> 固定高度航线。
#ifndef ONBOARD_MISSION_INPUT_H
#define ONBOARD_MISSION_INPUT_H

#include <cstddef>
#include <string>
#include <vector>

#include "onboard/offboard_pilot.h"

namespace ego_onboard
{
  // 一次开机只接受一个本地 KML 任务：wait_for_file=true 时先等文件出现，
  // 出现后立即读取、解析，通过即进入 READY
  // 上传方直接用新文件覆盖同一路径
  struct MissionInputConfig
  {
    // 注意：max_waypoints 只计航点本身，返回值会自动追加返航段，追加后总数仍受此上限约束。
    std::string file;                   // KML 路径（相对启动工作目录）
    bool wait_for_file = false;         // 文件不存在时是否一直等待（开机接任务模式）
    std::size_t max_bytes = 5 * 1024 * 1024; // 文件大小上限，超限直接拒绝
    std::size_t max_waypoints = 2000;    // includes the first takeoff point
    // 高度固定由配置给出（相对 HOME），KML 坐标里的高度分量被忽略。
    double constant_alt_m = 50.0;       // 全程固定相对高度
    double min_alt_m = 0.5;             // 固定高度必须大于此值
    double max_alt_m = 1000.0;          // 相对高度上限
  };

  // 校验通过后的任务：route[0] 是起飞点，之后是导航点
  struct PreparedMission
  {
    std::vector<GeoWaypoint> route;     // route[0] = takeoff, route[1..] = NAV
  };

  // 任务输入状态机：IDLE -> WAIT_KML -> LOADING_KML -> PARSING_KML -> READY / FAILED
  // 解析阶段完成几何与高度范围校验；任何一步失败都直接进入 FAILED 并停在终态。
  enum class MissionInputState
  {
    IDLE, WAIT_KML, LOADING_KML, PARSING_KML, READY, FAILED
  };
  const char *missionInputStateName(MissionInputState state);

  // 解析成功后只填 mission，不做任何设备操作；高度固定取 constant_alt_m。
  bool parseMissionKml(const std::string &xml, const MissionInputConfig &config,
                       PreparedMission &mission, std::string &error);

  class MissionInput
  {
  public:
    void start(const MissionInputConfig &config);
    void step();                      // 用内部单调时钟推进一步
    void step(double monotonic_now);  // Explicit clock for deterministic inbox tests.
    MissionInputState state() const { return state_; }
    const PreparedMission &mission() const { return mission_; }
    const std::string &error() const { return error_; }
  private:
    MissionInputConfig config_;
    MissionInputState state_ = MissionInputState::IDLE;
    PreparedMission mission_;
    std::string body_, error_;              // 文件内容 / 最近一次错误
    void rejectOrFail();                    // 读取/解析失败：直接进入 FAILED 终态
  };
}
#endif
