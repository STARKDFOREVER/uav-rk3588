#ifndef ONBOARD_KML_PLANNER_START_H
#define ONBOARD_KML_PLANNER_START_H
#include <onboard/mission_input.h>
namespace ego_onboard {
// 作用：把已校验的 KML 航线接到规划器上，并在允许的阶段提前预热轨迹；
// 坐标系与单位：KML 为经纬度（deg）+ 高度（m，相对起飞点）；这里统一转成局部 ENU（m），

// 把 KML 航线（经纬高，高度相对起飞点）转成局部 ENU：
inline bool bindKmlPlanner(const std::vector<GeoWaypoint> &route,const ego_mav::GeoOrigin &home,PlannerIO &planner) {
  // 至少两个点（起点 + 任务点）且 HOME 有效才可绑定
  if(route.size()<2||!home.valid)return false;
  std::vector<Eigen::Vector3d> enu;enu.reserve(route.size());
  // 逐点做经纬 -> ENU 平面转换，高度直接取相对起飞点高度
  for(const auto &w:route){auto xy=ego_mav::geoToEnuXY(w.lat_deg,w.lon_deg,home);enu.emplace_back(xy.x(),xy.y(),w.alt_rel_m);}
  // 把首段 A->B 交给规划器；仅两点时该段即最后一段
  planner.setLeg(enu[0],enu[1],enu.size()==2);return true;
}
// 预规划：只有在"链路建立"或"等地面站起飞"且航线/输入都就绪时才跑一拍，
inline bool preplanKml(bool route_bound,bool inputs_ready,Phase phase,double now,
                      const Eigen::Vector3d &position,PlannerIO &planner) {
  if(!route_bound||!inputs_ready)return false;
  if(phase!=Phase::LINK&&phase!=Phase::WAIT_GROUND_START)return false;
  // 预热：只跑一拍 step 并丢弃结果，目的是让规划器提前建图/出轨迹
  ego_planner::QuadrotorCommand preview;planner.step(now,position,preview);return true;
}
}
#endif
