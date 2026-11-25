#ifndef ONBOARD_ROUTE_UTILS_H
#define ONBOARD_ROUTE_UTILS_H

#include <cmath>
#include <string>
#include "onboard/offboard_pilot.h"

namespace ego_onboard
{
  // 航线几何收尾工具。把航线首点约定为起飞原点，确保航线回到起点
  // 坐标系与单位：经纬度 deg、高度 m（相对起飞点）；max_points 为含首点的最大航点数。
  inline bool ensureReturnToStart(std::vector<GeoWaypoint> &route, std::size_t max_points,
                                 bool &added, std::string &error)
  {
    added = false;
    // 至少要有"起点 + 一个任务点"，且不能超过航点上限
    if (route.size() < 2 || route.size() > max_points)
    { error = "Route must contain an origin and at least one task waypoint"; return false; }
    const auto &a = route.front();
    const auto &b = route.back();
    // 端点必须是有限值且在合法经纬范围内，避免把 NaN/越界值送进规划
    if (!std::isfinite(a.lat_deg) || !std::isfinite(a.lon_deg) || !std::isfinite(a.alt_rel_m) ||
        !std::isfinite(b.lat_deg) || !std::isfinite(b.lon_deg) || !std::isfinite(b.alt_rel_m) ||
        std::abs(a.lat_deg) > 90 || std::abs(b.lat_deg) > 90 ||
        std::abs(a.lon_deg) > 180 || std::abs(b.lon_deg) > 180)
    { error = "Invalid route endpoints"; return false; }
    const bool same = std::llround(a.lat_deg * 1e7) == std::llround(b.lat_deg * 1e7) &&
                      std::llround(a.lon_deg * 1e7) == std::llround(b.lon_deg * 1e7) &&
                      std::abs(a.alt_rel_m - b.alt_rel_m) < 0.01;
    // 首尾同点：只把末点对齐到首点消除尾差，不追加航点
    if (same) route.back() = route.front();
    else
    {
      // 追加返航点前再确认还有容量
      if (route.size() >= max_points)
      { error = "No waypoint capacity left for return to origin"; return false; }
      const GeoWaypoint origin = route.front();
      // 返航点即首点副本
      route.push_back(origin);
      added = true;
    }
    return true;
  }
}
#endif
