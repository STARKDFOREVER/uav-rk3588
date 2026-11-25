// EgoPlannerAdapter 实现：把雷达帧/位姿/航段吃进来，维护 insert-only 滚动栅格地图，
// 周期性地用 EGO（A* 初值 + B样条优化）产出局部轨迹，再交给 TrajectoryTracker 输出指令。

#include "onboard/ego_planner_adapter.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace ego_onboard
{

  namespace
  {
    double msSince(const std::chrono::steady_clock::time_point &t0)
    {
      return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
  } // namespace

  EgoPlannerAdapter::EgoPlannerAdapter() {}
  EgoPlannerAdapter::~EgoPlannerAdapter() {}

  void EgoPlannerAdapter::init(const EgoPlannerAdapterConfig &cfg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cfg_ = cfg;
    // 只存配置不建图；地图由 prepareMap（AsyncPlanner 启动时）按巡航高度创建
    clearState();
  }

  void EgoPlannerAdapter::prepareMap(double reference_height) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!grid_map_) buildMap(reference_height);
  }

  void EgoPlannerAdapter::clearState()
  {
    grid_map_.reset();
    z_ref_ = 0.0;

    route_.clear();
    route_total_ = 0.0;
    leg_last_ = false;

    // 轨迹、命令、定时全部归零
    invalidateTrajectory();
    have_cmd_ = false;

    pose_buf_ = ego_sensor::PoseBuffer();
    last_recenter_.setZero();

    stats_ = Stats();
    last_scan_time_ = -1e9;
    pose_buf_.setStrictCoverage(cfg_.strict_pose_coverage);
  }

  // 时钟不连续：位姿的绝对时间已不可信，清空位姿缓冲并丢弃当前轨迹。
  // 栅格地图保持不动——已插入的点都是世界系坐标，与时间映射无关。
  void EgoPlannerAdapter::clearPoseHistory() {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_buf_ = ego_sensor::PoseBuffer();
    pose_buf_.setStrictCoverage(cfg_.strict_pose_coverage);
    last_scan_time_ = -1e9;
    invalidateTrajectory();
    have_cmd_ = false;
  }

  // 轨迹失效只有一个实现：调用方必须已持有 mtx_
  void EgoPlannerAdapter::invalidateTrajectory() {
    have_traj_ = false;
    braking_ = true;
    collides_ahead_ = false;
    last_plan_ = -1e9;
    last_safety_ = -1e9;
    stats_.trajectory_active = false;
  }

  bool EgoPlannerAdapter::frameCovered(const ego_sensor::LidarFrame &frame) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return grid_map_ && scan_asm_.covered(frame, pose_buf_);
  }

  void EgoPlannerAdapter::buildMap(double z_ref)
  {
    z_ref_ = z_ref;

    // 固定地板模式的下界表示膨胀后的机体中心最低高度；旧配置保留自动模式。
    const double band_lo = cfg_.use_fixed_virtual_floor ? cfg_.virtual_floor_height :
        std::max(z_ref - cfg_.max_descent_below_ref, cfg_.min_flight_height);
    // 高度带 [band_lo, band_hi]：只在这层里做规划，带外视作无数据/不可达
    const double band_hi = z_ref + cfg_.altitude_band_half;

    // 地图 z 窗口 = 高度带再外扩 margin
    // 滚动窗口只平移 x/y
    const double map_z_lo = band_lo - cfg_.map_margin_z;
    const double z_size = (band_hi + cfg_.map_margin_z) - map_z_lo;

    grid_map_.reset(new GridMap);
    grid_map_->initMap(cfg_.resolution,
                       Eigen::Vector3d(cfg_.map_size_xy.x(), cfg_.map_size_xy.y(), z_size),
                       Eigen::Vector3d(cfg_.occupancy_update_range_xy, cfg_.occupancy_update_range_xy,
                                       cfg_.occupancy_update_range_z),
                       cfg_.obstacles_inflation,
                       band_hi + cfg_.map_margin_z, // 天花板：交给 initMap 夹到地图顶
                       map_z_lo,                    // 地图底
                       cfg_.local_map_margin, "world");
    // GridMap fills ground_id..ground_id+inf_step inclusively. Place the
    // backing plane below the desired free boundary by inflation plus one
    // voxel, so a configured 2 m floor becomes free at 2 m, not at 5 m.
    // 地板层要再下沉"膨胀格数 + 1 个体素"（magic），确保地板障碍层完全落在期望自由边界之下。
    const double floor_padding = (std::ceil(cfg_.obstacles_inflation / cfg_.resolution) + 1.0) * cfg_.resolution;
    grid_map_->setVirtualGroundHeight(cfg_.use_fixed_virtual_floor ? band_lo - floor_padding : band_lo);
    if (cfg_.use_fixed_virtual_floor)
      grid_map_->setMinimumCenterHeight(band_lo);

    ego_planner::EgoLocalPlannerConfig pcfg;
    pcfg.max_vel = cfg_.max_vel;
    pcfg.max_acc = cfg_.max_acc;
    pcfg.ctrl_pt_dist = cfg_.ctrl_pt_dist;
    pcfg.planning_horizon = cfg_.planning_horizon;
    pcfg.astar_pool_size = cfg_.astar_pool_size;
    // 优化代价权重（经验值）：平滑与拟合为主，碰撞次之，可行性作为软约束
    pcfg.opt.lambda_smooth = 1.0;
    pcfg.opt.lambda_collision = 0.5;
    pcfg.opt.lambda_feasibility = 0.1;
    pcfg.opt.lambda_fitness = 1.0;
    pcfg.opt.dist0 = cfg_.obstacle_clearance;
    pcfg.opt.swarm_clearance = cfg_.obstacle_clearance;
    pcfg.opt.max_vel = pcfg.max_vel;
    pcfg.opt.max_acc = pcfg.max_acc;

    planner_.init(pcfg);
    planner_.setEnvironment(grid_map_);

    tracker_ = ego_planner::TrajectoryTracker();
    tracker_.setTimeForward(cfg_.time_forward);
    scan_asm_.setExtrinsic(cfg_.extrinsic);

    // 轨迹状态清零
    invalidateTrajectory();
    have_cmd_ = false;
  }

  // 航段
  void EgoPlannerAdapter::setLeg(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool is_last)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    route_.assign({start, end});
    route_total_ = (end - start).norm();
    leg_last_ = is_last;

    // 新航段，旧轨迹作废；下一拍立即重规划。
    // 保留 last_cmd_，这样悬停/起步时的 yaw 不会跳变。
    invalidateTrajectory();
  }

  // ------------------------------------------------------------------ 输入
  void EgoPlannerAdapter::onPose(const ego_sensor::TimedPose &pose)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_buf_.add(pose);
  }

  void EgoPlannerAdapter::onScanFrame(const ego_sensor::LidarFrame &frame)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!grid_map_ || pose_buf_.empty())
      return;

    stats_.frames_total++;

    // 1) 逐点去畸变 -> 世界系点云。位姿覆盖不了整帧时必须丢帧：
    //    insert-only 地图会把用错位姿建出来的点永久留下。
    std::vector<Eigen::Vector3d> cloud;
    if (!scan_asm_.toWorld(frame, pose_buf_, cloud) || cloud.empty())
    {
      stats_.frames_dropped++;
      ++stats_.consecutive_dropped;
      if (stats_.consecutive_dropped >= cfg_.sensing_drop_frames)
        stats_.sensing_dead = true; // 连续丢帧：地图不再更新，避障已失效
      return;
    }
    // 一帧成功就算恢复（上层还会再做一次时间确认）
    stats_.consecutive_dropped = 0;
    stats_.sensing_dead = false;
    last_scan_time_ = frame.t_base + frame.points.back().t_rel;

    // 2) 取该帧基准时刻的位姿当作观测点（一帧 100 ms，用最新位姿会偏）
    ego_sensor::TimedPose tp;
    if (!pose_buf_.poseAt(frame.t_base, tp))
      pose_buf_.latest(tp); // 极端情况下基准时刻无覆盖，退回最新位姿
    const Eigen::Vector3d view = tp.p;

    // 3) 滚动窗口跟着机体平移
    //    仅当横向位移超过 recenter_margin 才搬图，避免高频搬迁带来的额外开销。
    if ((view.head<2>() - last_recenter_.head<2>()).norm() > cfg_.recenter_margin)
    {
      grid_map_->recenter(view);
      last_recenter_ = view;
      stats_.recenter++;
    }

    // 4) 插入（insert-only：只把 0 写成 1，不做逐帧清空）
    grid_map_->sensedPoints(cloud, view);
    stats_.cloud_points += (long)cloud.size();
    // 实测插入耗时：决定 update_range / inflation 能开多大
    stats_.inflate_ms_sum += grid_map_->prof_inflate_ms;
    stats_.inflate_ms_max = std::max(stats_.inflate_ms_max, grid_map_->prof_inflate_ms);
    ++stats_.inflate_samples;
  }

  // ------------------------------------------------------------- 轨迹管理
  bool EgoPlannerAdapter::sensingHealthy() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    // 健康条件：进过点、未连续丢帧、位姿时间"追上"点云时间（容差 0.02 s）且延迟不超时
    if (pose_buf_.empty() || stats_.inflate_samples <= 0 || stats_.sensing_dead)
      return false;
    const double newest_pose_time = pose_buf_.newestTime();
    return newest_pose_time >= last_scan_time_ - 0.02 &&
           newest_pose_time - last_scan_time_ <= cfg_.sensing_timeout_s;
  }

  void EgoPlannerAdapter::dropTrajectory()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    invalidateTrajectory();
  }

  // ------------------------------------------------------------- 航线几何
  double EgoPlannerAdapter::projectRoute(const Eigen::Vector3d &p, double &s_out) const
  {
    // 退化情况：航线不足两点时直接返回到首点的距离，1e18 作为"无穷远"哨兵
    if (route_.size() < 2)
    {
      s_out = 0.0;
      return route_.empty() ? 1e18 : (p - route_[0]).norm();
    }

    // 单线段投影：把 p 投到 A->B 上并夹到端点，输出弧长
    const Eigen::Vector3d ab = route_[1] - route_[0];
    const double len2 = ab.squaredNorm();
    s_out = 0.0;
    if (len2 > 1e-12)
    {
      const double u = std::max(0.0, std::min(1.0, (p - route_[0]).dot(ab) / len2));
      s_out = u * route_total_;
    }
    return (p - routeAt(s_out)).norm();
  }

  Eigen::Vector3d EgoPlannerAdapter::routeAt(double s) const
  {
    if (route_.empty())
      return Eigen::Vector3d::Zero();
    if (route_.size() < 2 || route_total_ <= 1e-9)
      return route_.front();
    s = std::max(0.0, std::min(route_total_, s));
    return route_[0] + (route_[1] - route_[0]) * (s / route_total_);
  }

  Eigen::Vector3d EgoPlannerAdapter::routeDirAt(double) const
  {
    if (route_.size() < 2)
      return Eigen::Vector3d(1.0, 0.0, 0.0);
    const Eigen::Vector3d d = route_[1] - route_[0];
    const double n = d.norm();
    return n > 1e-9 ? d / n : Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  Eigen::Vector3d EgoPlannerAdapter::localTargetAt(double s_cur, double &s_target) const
  {
    // 前视上限：既不能超过"让步倍数"，也不能超过地图真正有数据的范围
    const double lt_max =
        std::min(cfg_.local_target_max_ratio * cfg_.planning_horizon, cfg_.occupancy_update_range_xy);

    double la = cfg_.planning_horizon;
    double s_t = std::min(s_cur + la, route_total_);
    Eigen::Vector3d p = routeAt(s_t);

    // 落点是"已知被占"（==1）就沿航线继续前走；-1（窗口外=未知）不当障碍
    while (s_cur + la < route_total_ && la < lt_max && grid_map_->getInflateOccupancy(p) == 1)
    {
      la = std::min(la + cfg_.local_target_step, lt_max);
      s_t = std::min(s_cur + la, route_total_);
      p = routeAt(s_t);
    }

    // 还是被占（障碍比让步范围还宽 / 正压在航线上）：整体抬升，从上方越过。
    // 抬升上限跟着高度带的基准走，再留一个膨胀量，避免目标点贴到天花板板层上。
    if (grid_map_->getInflateOccupancy(p) == 1)
    {
      const double z_top = z_ref_ + cfg_.altitude_band_half - cfg_.obstacles_inflation;
      for (double z = p.z() + cfg_.local_target_step; z <= z_top; z += cfg_.local_target_step)
      {
        const Eigen::Vector3d q(p.x(), p.y(), z);
        if (grid_map_->getInflateOccupancy(q) != 1)
        {
          p = q;
          break;
        }
      }
    }

    s_target = s_t;
    return p;
  }

  bool EgoPlannerAdapter::trajCollidesAhead(double now)
  {
    if (!have_traj_)
      return false;
    // 只查前 2/3 时长（magic）：覆盖近期真正会被跟踪的飞行段，尾段多为减速收尾；
    // 采样步长按地图分辨率取，保证逐格检查不漏障。
    const double t_cur = std::max(0.0, now - traj_.start_time);
    const double t_23 = traj_.duration * 2.0 / 3.0;
    double t_end = (t_cur < t_23) ? t_23 : traj_.duration;
    const double t_step = traj_.bspline.collisionCheckInterval(cfg_.resolution);
    for (double tt = t_cur; tt <= t_end + 1e-6; tt += t_step)
    {
      const Eigen::Vector3d q = traj_.bspline.evaluateDeBoorT(std::min(tt, traj_.duration));
      if (grid_map_->getInflateOccupancy(q) != 0)
        return true;
    }
    return false;
  }

  // ---------------------------------------------------------------- 规划
  bool EgoPlannerAdapter::step(double now, const Eigen::Vector3d &cur_pos,
                               ego_planner::QuadrotorCommand &cmd)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    // 悬停保底：保持当前位置、零速零加速，yaw 沿用上一条指令避免跳变
    auto hold = [&]() {
      ++stats_.hold_commands;
      stats_.trajectory_active = false;
      cmd.position = cur_pos;
      cmd.velocity.setZero();
      cmd.acceleration.setZero();
      cmd.yaw = have_cmd_ ? last_cmd_.yaw : 0.0;
      cmd.yaw_dot = 0.0;
      cmd.stamp = now;
      cmd.trajectory_flag = ego_planner::TRAJECTORY_STATUS_EMPTY;
      last_cmd_ = cmd;
      have_cmd_ = true;
    };

    // 地图由 prepareMap 在启动时创建；未创建时只能保持（不在这里重复建图）
    if (!grid_map_)
    {
      hold();
      return true;
    }

    ++stats_.step_calls;
    stats_.start_blocked = grid_map_->getInflateOccupancy(cur_pos) != 0;
    if (stats_.start_blocked && !have_traj_)
    {
      // 起点落在膨胀障碍里属于输入/地图条件问题：直接保持，不浪费一次优化。
      // Invalid initial start is an input/map condition: hold rather than
      // spending time optimizing a trajectory from inside an inflated obstacle.
      ++stats_.blocked_start_holds;
      have_traj_ = false;
      braking_ = true;
      hold();
      return true;
    }

    if (route_.size() < 2)
    {
      braking_ = true;
      hold();
      return true;
    }

    double s_cur = 0.0;
    projectRoute(cur_pos, s_cur);

    double s_tgt = s_cur;
    const Eigen::Vector3d local_target = localTargetAt(s_cur, s_tgt);

    // 局部目标已经到达（最后一段时就是任务终点）：停在原地，由上层 OffboardPilot
    // 判点后推进航段 / 交还飞控。不再重规划，否则会不停报 "Close to goal"。
    if ((cur_pos - local_target).norm() < cfg_.goal_tolerance)
    {
      have_traj_ = false;
      braking_ = true;
      hold();
      return true;
    }

    // 轨迹安全扫描（20 Hz，同原 FSM 的 safety 定时器）
    if (have_traj_ && now - last_safety_ >= 0.05)
    {
      collides_ahead_ = trajCollidesAhead(now);
      last_safety_ = now;
    }

    // 重规划条件：无轨迹时 0.1 s 限速重试；有轨迹时看前方碰撞、
    // 周期到点（replan_period）或轨迹剩余寿命不足 0.2 s。
    bool need_plan;
    if (!have_traj_)
      need_plan = (now - last_plan_ >= 0.1); // 还没轨迹时限速重试
    else
      need_plan = collides_ahead_ || (now - last_plan_ >= cfg_.replan_period) ||
                  (traj_.start_time + traj_.duration - now <= 0.2);

    if (need_plan)
    {
      // 前视点的期望通过速度独立于样条限速；最后航段继续按剩余距离减速。
      const double pass_speed = std::min(cfg_.local_target_speed, cfg_.max_vel);
      const double remaining = std::max(0.0, route_total_ - s_tgt);
      const double target_speed = leg_last_
          ? std::min(pass_speed, std::sqrt(2.0 * cfg_.max_acc * remaining))
          : pass_speed;
      const Eigen::Vector3d local_target_vel = routeDirAt(s_tgt) * target_speed;

      // 起点状态：用上一次输出的速度/加速度（EGO 自己的输出 ≈ 实际状态，
      // 比用位置差分稳），第一次规划则为零。
      const Eigen::Vector3d v0 = have_cmd_ ? last_cmd_.velocity : Eigen::Vector3d::Zero();
      const Eigen::Vector3d a0 = have_cmd_ ? last_cmd_.acceleration : Eigen::Vector3d::Zero();

      ego_planner::BsplineTrajectory new_traj;
      // 首帧 = FSM GEN_NEW_TRAJ，之后 = FSM REPLAN_TRAJ
      const auto mode = have_traj_ ? ego_planner::PlanMode::REPLAN_TRAJ : ego_planner::PlanMode::NEW_TRAJ;
      const auto t0 = std::chrono::steady_clock::now();
      const bool ok = planner_.plan(cur_pos, v0, a0, local_target, local_target_vel, now, new_traj, mode);
      const auto optimizer_stats = planner_.optimizerStats();
      stats_.ego_optimizations = optimizer_stats.rebound_calls + optimizer_stats.refine_calls;
      stats_.astar_searches = optimizer_stats.astar_calls;
      stats_.astar_successes = optimizer_stats.astar_successes;
      stats_.last_plan_ms = msSince(t0);
      stats_.plan_ms_max = std::max(stats_.plan_ms_max, stats_.last_plan_ms);

      if (ok)
      {
        ++stats_.plan_success;
        traj_ = new_traj;
        tracker_.setTrajectory(traj_);
        have_traj_ = true;
        braking_ = false;
      }
      else
      {
        stats_.plan_fail++;
        // 规划不出来就原地悬停，绝不硬闯
        if (!have_traj_ || collides_ahead_ || (traj_.start_time + traj_.duration - now <= 0.2))
          braking_ = true;
      }
      last_plan_ = now;
      stats_.replan++;
    }

    if (!have_traj_)
      braking_ = true;

    if (braking_)
    {
      hold();
      return true;
    }

    if (!tracker_.getCommand(now, cmd))
    {
      braking_ = true;
      hold();
      return true;
    }

    // 成功从跟踪器取到当拍 setpoint：缓存为下一拍起点状态并标记轨迹有效
    last_cmd_ = cmd;
    have_cmd_ = true;
    ++stats_.tracking_commands;
    stats_.trajectory_active = true;
    return true;
  }

} // namespace ego_onboard
