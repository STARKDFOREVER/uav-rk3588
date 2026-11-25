#include "onboard/offboard_pilot.h"
#include "onboard/route_utils.h"
#include <cmath>
#include <algorithm>
#include <string>

// Offboard 飞行主控实现：MAVLink 收发 + 状态机 + setpoint 输出。

namespace ego_onboard
{

  const char *phaseName(Phase p)
  {
    switch (p)
    {
    case Phase::IDLE: return "IDLE";
    case Phase::LINK: return "LINK";
    case Phase::WAIT_GROUND_START: return "WAIT_GROUND_START";
    case Phase::PRE_OFFBOARD: return "PRE_OFFBOARD";
    case Phase::OFFBOARD_CRUISE: return "OFFBOARD_CRUISE";
    case Phase::HOLD: return "HOLD";
    case Phase::FAILED: return "FAILED";
    }
    return "?";
  }

  const char *holdReasonName(HoldReason r)
  {
    switch (r)
    {
    case HoldReason::NONE: return "NONE";
    case HoldReason::GPS_LOST: return "GPS_LOST";
    case HoldReason::SENSOR_LOST: return "SENSOR_LOST";
    case HoldReason::MISSION_DONE: return "MISSION_DONE";
    }
    return "?";
  }

  void OffboardPilot::fail(const std::string &why)
  {
    err_ = why;
    phase_ = Phase::FAILED;
  }

  // 启动
  void OffboardPilot::start(const std::vector<GeoWaypoint> &route)
  {
    // 入口校验：航点数至少 2，高度必须有限且为正；任何一项不满足直接 FAILED，不进入 LINK。
    route_ = route;
    if (route_.size() < 2 ||
        !std::isfinite(cfg_.cruise_alt_m) || cfg_.cruise_alt_m <= 0.5 ||
        !std::isfinite(cfg_.handover_alt_tolerance_m) || cfg_.handover_alt_tolerance_m <= 0)
    {
      fail("invalid route size / cruise handover height");
      return;
    }
    for (const auto &w : route_)
      if (!std::isfinite(w.lat_deg) || !std::isfinite(w.lon_deg) ||
          !std::isfinite(w.alt_rel_m) || std::abs(w.lat_deg) > 90 ||
          std::abs(w.lon_deg) > 180 || w.alt_rel_m <= 0.5)
      {
        fail("invalid route coordinate / height");
        return;
      }
    bool added = false;
    std::string why;
    if (!ensureReturnToStart(route_, added, why))
    {
      fail(why);
      return;
    }
    leg_ = 0;
    planner_setpoints_ = cruise_hold_setpoints_ = 0;
    mode_switch_sent_ = false;
    waiting_offboard_ = false;
    mode_ack_seen_ = false;
    mode_ack_result_ = 255;
    t_last_mode_ack_ = -1e9;
    t_handover_candidate_ = -1e9;
    err_.clear();
    phase_ = Phase::LINK;
    t_start_ = -1.0;
    t_phase_ = 0.0;

    // GPS 健康状态
    gps_fix_type_ = 0;
    t_last_gps_ = -1e9;
    t_gps_bad_ = -1e9;
    // setpoint 的 type_mask：位置 + yaw 必发；速度前馈按配置
    {
      // bit3..5 忽略速度、bit6..8 忽略加速度、bit11 忽略 yaw_rate；
      // 置 1 表示该维不参与控制，PX4 会直接忽略对应字段。
      const uint16_t kIgnoreAcc = (uint16_t)((1 << 6) | (1 << 7) | (1 << 8));
      const uint16_t kIgnoreYawRate = (uint16_t)(1 << 11);
      const uint16_t kIgnoreVel = (uint16_t)((1 << 3) | (1 << 4) | (1 << 5));
      sp_cfg_.type_mask = (uint16_t)(kIgnoreAcc | kIgnoreYawRate |
                                     (cfg_.velocity_feedforward ? 0 : kIgnoreVel));
    }

    // 安全悬停状态
    hold_pos_.setZero();
    hold_reason_ = HoldReason::NONE;
    t_hold_clear_ = -1e9;
    sensing_healthy_ = true;
    reckoner_.reset();
  }

  // ---------------------------------------------------------------- 接收
  void OffboardPilot::onMavlinkBytes(const uint8_t *data, size_t len)
  {
    MavMessage msg;
    for (size_t i = 0; i < len; ++i)
    {
      if (!parser_.push(data[i], msg))
        continue;
      // 只信任 CRC 通过、且来自目标飞控的帧；GCS 与其他组件一律丢弃。
      if (!msg.crc_checked || msg.sysid != cfg_.target_system || msg.compid != cfg_.target_component) continue;
      switch (msg.msgid)
      {
      case Timesync::ID: {
        // tc1>0：飞控发起，用往返时延估计校时偏差；tc1==0：对方在等回应，
        // 这里用本机时刻填 tc1 原样回发，完成一次对称测量。
        Timesync m;
        if (Timesync::decode(msg.payload.data(), msg.payload.size(), m) &&
            (!m.target_system || m.target_system == cfg_.sysid) &&
            (!m.target_component || m.target_component == cfg_.compid)) {
          if (m.tc1 > 0) alignment_.accept(m.tc1, m.ts1, t_now_);
          else if (m.tc1 == 0) {
            m.tc1 = int64_t(t_now_ * 1e9);
            send(m, outbox_);
          }
        }
        break;
      }
      case Heartbeat::ID:
      {
        Heartbeat hb;
        if (Heartbeat::decode(msg.payload.data(), msg.payload.size(), hb))
          onHeartbeat(hb, t_now_);
        break;
      }
      case HomePosition::ID:
      {
        HomePosition hp;
        if (HomePosition::decode(msg.payload.data(), msg.payload.size(), hp))
          onHomePosition(hp);
        break;
      }
      case GlobalPositionInt::ID:
      {
        GlobalPositionInt g;
        if (GlobalPositionInt::decode(msg.payload.data(), msg.payload.size(), g))
          onGlobalPositionInt(g);
        break;
      }
      case Attitude::ID:
      {
        Attitude a;
        if (Attitude::decode(msg.payload.data(), msg.payload.size(), a))
          onAttitude(a);
        break;
      }
      case GpsRawInt::ID:
      {
        GpsRawInt g;
        if (GpsRawInt::decode(msg.payload.data(), msg.payload.size(), g))
          onGpsRawInt(g);
        break;
      }
      case CommandAck::ID:
      {
        CommandAck ak;
        if (CommandAck::decode(msg.payload.data(), msg.payload.size(), ak))
          onCommandAck(ak);
        break;
      }
      default:
        break;
      }
    }
  }

  void OffboardPilot::onHeartbeat(const Heartbeat &hb, double now)
  {
    // 只认飞控（非 GCS 类型）的心跳
    if (hb.type == 6)
      return;
    armed_ = hb.armed();
    main_mode_ = hb.mainMode();
    sub_mode_ = hb.subMode();
    t_last_fc_hb_ = now;
  }

  void OffboardPilot::onHomePosition(const HomePosition &hp)
  {
    // 经纬度单位 1e-7 度。HOME 一旦确定就不能再变：地图像素是原点的函数，换了原点旧地图即失效。
    if (std::abs(hp.latitude * 1e-7)>90 || std::abs(hp.longitude * 1e-7)>180) return;
    if (origin_.valid && (std::abs(origin_.lat_deg-hp.latitude*1e-7)>1e-7 || std::abs(origin_.lon_deg-hp.longitude*1e-7)>1e-7)) {
      fail("HOME changed: restart with a consistent map origin"); return;
    }
    origin_.lat_deg = hp.latitude * 1e-7;
    origin_.lon_deg = hp.longitude * 1e-7;
    origin_.valid = true;
    pose_asm_.setOrigin(origin_);
  }

  void OffboardPilot::onGlobalPositionInt(const GlobalPositionInt &g)
  {
    if (!origin_.valid)
      return;
    const double boot = g.time_boot_ms * 0.001;
    // time_boot_ms 回退超过 50ms = 飞控重启或时钟被重设：清校时、作废位姿，
    // 不能再用旧时刻做判据。
    if (last_pos_boot_ >= 0 && boot < last_pos_boot_ - 0.05) { alignment_.reset(); have_pos_ = false; fail("FCU boot clock reset"); return; }
    if (boot <= last_pos_boot_) return;
    last_pos_boot_ = boot; t_last_pos_ = t_now_;
    if (std::abs(g.lat * 1e-7)>90 || std::abs(g.lon * 1e-7)>180) { have_pos_=false; return; }
    pose_asm_.onGlobalPositionInt(g);
    pos_enu_ = pose_asm_.positionEnu();
    have_pos_ = true;

    // 航位推算：定位有效就用它重置锚点；无效则对速度做一次积分（输出经纬高给上报用）
    if (gpsFixFresh(t_now_))
    {
      GeoFix f;
      f.lat_deg = g.lat * 1e-7;
      f.lon_deg = g.lon * 1e-7;
      f.alt_m = g.relative_alt * 1e-3;
      reckoner_.onValidFix(f, t_now_);
    }
    else
    {
      reckoner_.onVelocity(pose_asm_.velocityEnu(), t_now_);
    }
  }

  void OffboardPilot::onGpsRawInt(const GpsRawInt &g)
  {
    // 这里只记录原始状态，不做判定：定位是否可信统一在 evaluateHealth 里按
    // 「报文仍在来 + fix_type 够好」两个条件计算，避免多处口径不一致。
    gps_fix_type_ = g.fix_type;
    t_last_gps_ = t_now_;
  }

  void OffboardPilot::onAttitude(const Attitude &a)
  {
    if (!std::isfinite(a.roll) || !std::isfinite(a.pitch) || !std::isfinite(a.yaw)) return;
    const double boot = a.time_boot_ms * 0.001;
    if (last_att_boot_ >= 0 && boot < last_att_boot_ - 0.05) { alignment_.reset(); fail("FCU attitude clock reset"); return; }
    if (boot <= last_att_boot_) return;
    last_att_boot_ = boot; t_last_att_ = t_now_;
    // 三道门：校时未收敛 / 映射到宿主时钟后已过期 / 位置流已停止外推；任一成立就不产出位姿。
    if (!alignment_.valid(t_now_) || std::abs(alignment_.hostTime(boot) - t_now_) > cfg_.pose_timeout_s) return;
    if (t_now_ - t_last_pos_ > cfg_.position_extrapolation_s) return;
    ego_sensor::TimedPose tp;
    if (!pose_asm_.onAttitude(a, boot, tp)) return;
    tp.t=alignment_.hostTime(boot);
    // 位姿和定位锚点必须使用同一个时钟映射，逐点查询才可继续按速度外推。
    tp.position_anchor.t += tp.t - boot;
    if (t_now_-tp.t>cfg_.pose_timeout_s || tp.t>t_now_+0.005) return;
    hold_yaw_ = std::atan2(tp.q.toRotationMatrix()(1,0),tp.q.toRotationMatrix()(0,0));
    latest_pose_=tp;
    if(planner_)planner_->onPose(tp);
  }

  void OffboardPilot::onCommandAck(const CommandAck &ack)
  {
    // Only Offboard mode switching is owned by this controller. Ignore ACKs
    // for arm/takeoff/landing commands sent by the independent ground station.
    if (ack.command != CMD_DO_SET_MODE || !mode_switch_sent_) return;
    mode_ack_seen_ = true;
    mode_ack_result_ = ack.result;
    t_last_mode_ack_ = t_now_;
  }

  void OffboardPilot::sendHeartbeat(std::vector<std::vector<uint8_t>> &out, double now)
  {
    Heartbeat hb; // 默认即 GCS 心跳：type=6 autopilot=8 base_mode=192 status=4
    send(hb, out);
    t_last_hb_ = now;
  }

  bool OffboardPilot::gpsFixFresh(double now) const
  {
    return t_last_gps_ > -1e8 && now - t_last_gps_ <= cfg_.gps_rx_timeout_s &&
        gps_fix_type_ >= cfg_.gps_min_fix;
  }

  // ----------------------------------------------------------- 单拍健康快照
  OffboardPilot::Health OffboardPilot::evaluateHealth(double now) const
  {
    Health h;
    h.fc_alive = t_last_fc_hb_ > -1e8 && now - t_last_fc_hb_ < cfg_.link_timeout;
    h.pose_ok = poseFresh(now);
    h.timing_ok = clockReady(now) && input_ready_;
    h.gps_rx = t_last_gps_ > -1e8 && now - t_last_gps_ <= cfg_.gps_rx_timeout_s;
    h.gps_ok = !cfg_.gps_failsafe || gpsFixFresh(now);
    h.sensing_ok = sensing_healthy_;
    h.all_clear = h.gps_ok && h.sensing_ok && h.timing_ok && h.pose_ok;
    if (!route_enu_.empty())
      h.altitude_ok = std::abs(pos_enu_.z() - cfg_.cruise_alt_m) <=
          cfg_.handover_alt_tolerance_m;
    return h;
  }

  // 输出
  bool OffboardPilot::emitSetpoint(double now, const ego_planner::QuadrotorCommand &cmd,
                                   std::vector<std::vector<uint8_t>> &out)
  {
    if (!cmd.position.allFinite() || !cmd.velocity.allFinite() || !cmd.acceleration.allFinite() ||
        !std::isfinite(cmd.yaw) || !std::isfinite(cmd.yaw_dot))
      return false;
    SetPositionTargetLocalNed sp;
    toSetpoint(cmd, alignment_.bootMs(now), sp_cfg_, sp);
    sp.target_system = cfg_.target_system;
    sp.target_component = cfg_.target_component;
    send(sp, out);
    t_last_setpoint_ = now;
    return true;
  }

  bool OffboardPilot::emitHoldSetpoint(double now, const Eigen::Vector3d &pos,
                                       std::vector<std::vector<uint8_t>> &out)
  {
    ego_planner::QuadrotorCommand cmd;
    cmd.position = pos;
    cmd.velocity.setZero();
    cmd.acceleration.setZero();
    cmd.yaw = hold_yaw_;
    cmd.yaw_dot = 0.0;
    return emitSetpoint(now, cmd, out);
  }

  void OffboardPilot::requestRatesOnce(std::vector<std::vector<uint8_t>> &out)
  {
    // 提频只发一次（req_stream_id 填消息 ID），并附带 245 EXTENDED_SYS_STATE；
    if (!cfg_.request_rates || rates_requested_)
      return;
    rates_requested_ = true;
    send(RequestDataStream::forMessage(GlobalPositionInt::ID, cfg_.rate_global_position_hz, cfg_.target_system), out);
    send(RequestDataStream::forMessage(Attitude::ID, cfg_.rate_attitude_hz, cfg_.target_system), out);
    send(RequestDataStream::forMessage(245, cfg_.rate_extension_hz, cfg_.target_system), out);
  }

  HandoverDiagnostics OffboardPilot::handoverDiagnostics(double now) const
  {
    return handoverDiagnostics(now, evaluateHealth(now));
  }

  HandoverDiagnostics OffboardPilot::handoverDiagnostics(double now, const Health &h) const
  {
    HandoverDiagnostics d;
    d.fc_alive = h.fc_alive;
    d.armed = armed_;
    d.main_mode = main_mode_;
    d.sub_mode = sub_mode_;
    d.pose_ok = h.pose_ok;
    d.timing_ok = h.timing_ok;
    d.sensing_ok = h.sensing_ok;
    d.gps_rx = h.gps_rx;
    d.gps_fix_type = gps_fix_type_;
    d.gps_ok = h.gps_ok;
    d.route_valid = !route_enu_.empty();
    d.altitude_rel_m = pos_enu_.z();
    d.speed_mps = pose_asm_.velocityEnu().norm();
    d.phase_elapsed_s = std::max(0.0, now - t_phase_);
    d.ready_stable_s = t_handover_candidate_ < -1e8 ? 0.0 :
        std::max(0.0, now - t_handover_candidate_);
    d.mode_switch_sent = mode_switch_sent_;
    d.waiting_offboard = waiting_offboard_;
    d.mode_ack_seen = mode_ack_seen_;
    d.mode_ack_result = mode_ack_result_;
    if (mode_ack_seen_)
      d.mode_ack_age_s = std::max(0.0, now - t_last_mode_ack_);

    if (d.route_valid)
    {
      d.horizontal_error_m =
          (pos_enu_.head<2>() - route_enu_.front().head<2>()).norm();
      d.horizontal_position_ok = d.horizontal_error_m <= cfg_.handover_position_tolerance_m;
    }
    d.altitude_ok = h.altitude_ok; // 无航线时 evaluateHealth 已给 false
    d.speed_ok = d.speed_mps <= cfg_.handover_max_speed_mps;
    // 复用 evaluateHealth 的汇总判定，避免两套口径；接管额外要求已解锁 + 定点模式。
    d.ready = h.all_clear && d.fc_alive && d.armed && d.main_mode == MODE_POS_HOLD &&
        d.horizontal_position_ok && d.altitude_ok && d.speed_ok;
    return d;
  }

  // ---------------------------------------------------------------- 周期
  void OffboardPilot::tick(double now, std::vector<std::vector<uint8_t>> &out)
  {
    t_now_ = now;
    if (t_start_ < 0)
      t_start_ = now;

    // 先吐出接收路径上攒下的帧
    for (auto &f : outbox_)
      out.push_back(std::move(f));
    outbox_.clear();

    // 心跳
    if (now - t_last_hb_ >= 1.0 / cfg_.heartbeat_hz)
      sendHeartbeat(out, now);

    if (now - t_last_sync_request_ >= 0.2) {
      Timesync m; m.ts1 = alignment_.request(now);
      send(m, out); t_last_sync_request_ = now;
    }
    if (!origin_.valid && now - t_last_home_request_ >= 2.0) {
      CommandLong request; request.command=512; request.target_system=cfg_.target_system;
      request.target_component=cfg_.target_component; request.param1=242;
      send(request,out); t_last_home_request_=now;
    }
    // 只有真正在控制飞机时才要求时钟连续；LINK/等待阶段断一下可以重新收敛。
    const bool active = phase_ == Phase::OFFBOARD_CRUISE || phase_ == Phase::HOLD;
    if (active && !clockReady(now)) { fail("FCU clock synchronization lost"); return; }

    // 单拍健康快照
    sensing_healthy_ = !cfg_.sensing_failsafe || !planner_ || planner_->sensingHealthy();
    const Health health = evaluateHealth(now);
    if (active && !health.pose_ok) { fail("position/attitude stream stale"); return; }

    // GPS 判据只有两个：GPS_RAW_INT 还在来（2Hz）、且 fix_type 够好。
    if (health.gps_ok) t_gps_bad_ = -1e9;
    else if (t_gps_bad_ < -1e8) t_gps_bad_ = now; // 连续无效的起始时刻

    // 统一恢复判据：所有异常都清除了，并且稳定一段时间
    if (health.all_clear)
    {
      if (t_hold_clear_ < -1e8)
        t_hold_clear_ = now;
    }
    else
    {
      t_hold_clear_ = -1e9;
    }

    if (phase_ == Phase::IDLE || phase_ == Phase::FAILED)
      return;

    if (active && now - t_last_setpoint_ < 1.0 / cfg_.setpoint_hz) return;
    switch (phase_)
    {
    // S0 链路
    case Phase::LINK:
    {
      if (health.fc_alive) requestRatesOnce(out);
      if (!health.fc_alive || !origin_.valid || !have_pos_)
      {
        if (now - t_start_ > 30.0)
          fail("link/home/position timeout");
        break;
      }
      // 参考航线（ENU，原点 = home）
      route_enu_.clear();
      for (const auto &w : route_)
      {
        const Eigen::Vector2d xy = geoToEnuXY(w.lat_deg, w.lon_deg, origin_);
        route_enu_.push_back(Eigen::Vector3d(xy.x(), xy.y(), w.alt_rel_m));
      }
      if (planner_)
      {
        if (route_enu_.size() >= 2)
          planner_->setLeg(route_enu_[0], route_enu_[1], route_enu_.size() == 2);
      }
      // Route and waypoint progress stay onboard; the FCU mission protocol is not used.
      phase_ = Phase::WAIT_GROUND_START;
      t_phase_ = now;
      break;
    }

    case Phase::WAIT_GROUND_START:
    {
      // 解锁/起飞由地面站完成；这里只等飞机稳定悬停在接管窗内，连续 handover_stable_s 才接管，
      // 中途任一项不满足就清零重新计时。
      const auto handover = handoverDiagnostics(now, health);
      if (handover.ready)
      {
        if (t_handover_candidate_ < -1e8) t_handover_candidate_ = now;
        if (now - t_handover_candidate_ >= cfg_.handover_stable_s)
        {
          phase_ = Phase::PRE_OFFBOARD;
          t_phase_ = now;
          t_handover_candidate_ = -1e9;
          mode_switch_sent_ = false;
        }
      }
      else t_handover_candidate_ = -1e9;
      break;
    }

    // S4 先连发 setpoint，再切 Offboard
    case Phase::PRE_OFFBOARD:
    {
      // 预发 setpoint 期间若飞机离开高度窗、出现异常或掉出定点/Offboard，退回等待阶段重新计时。
      const bool outside_handover_window = !mode_switch_sent_ && !health.altitude_ok;
      if (!health.all_clear || !armed_ || outside_handover_window ||
          (main_mode_ != MODE_POS_HOLD && main_mode_ != MODE_OFFBOARD))
      {
        // Restart the hover check if the aircraft leaves the height window before the request.
        phase_ = Phase::WAIT_GROUND_START;
        t_phase_ = now;
        mode_switch_sent_ = false;
        waiting_offboard_ = false;
        break;
      }
      if (now - t_last_setpoint_ < 1.0 / cfg_.setpoint_hz) break;
      // 从当前位姿起步（速度/加速度为 0），避免切模式瞬间位置跳变
      if (!emitHoldSetpoint(now, pos_enu_, out)) { fail("nonfinite control target"); break; }

      if (!mode_switch_sent_ && now - t_phase_ >= cfg_.pre_offboard_stream_s)
      {
        send(CommandLong::setMode(MODE_OFFBOARD, 0, cfg_.target_system), out);
        t_cmd_sent_ = now;
        mode_switch_sent_ = true;
        waiting_offboard_ = true;
        mode_ack_seen_ = false;
        mode_ack_result_ = 255;
        t_last_mode_ack_ = -1e9;
      }
      if (waiting_offboard_ && main_mode_ == MODE_OFFBOARD)
      {
        // 以飞控回报的主模式为准（而不是依赖 COMMAND_ACK），确认已进入 Offboard 才算接管成功。
        waiting_offboard_ = false;
        phase_ = Phase::OFFBOARD_CRUISE;
        t_phase_ = now;
      }
      else if (mode_switch_sent_ && now - t_cmd_sent_ > 3.0)
        fail("enter offboard timeout");
      break;
    }

    // S5 航点间 Offboard（我们的系统接管）
    case Phase::OFFBOARD_CRUISE:
    {
      // 飞控失联：此时没法悬停，只能报错（不能一直往断掉的链路写 setpoint）
      if (!health.fc_alive)
      {
        fail("fcu link lost");
        break;
      }
      if (main_mode_ != MODE_OFFBOARD)
      {
        fail("dropped out of offboard");
        break;
      }

      // 飞行任务异常，统一归到安全悬停 
      HoldReason why = HoldReason::NONE;
      if (!health.gps_ok && now - t_gps_bad_ >= cfg_.gps_loss_confirm_s)
        why = HoldReason::GPS_LOST;
      else if (!health.sensing_ok || !health.timing_ok)
        why = HoldReason::SENSOR_LOST;

      if (why != HoldReason::NONE)
      {
        hold_pos_ = latest_pose_.p; // 以当前位姿为悬停点
        hold_reason_ = why;
        t_hold_clear_ = -1e9; // 恢复计时从进入悬停重新开始，不能沿用之前的
        phase_ = Phase::HOLD;
        t_phase_ = now;
        break;
      }

      ego_planner::QuadrotorCommand cmd;
      bool ok = false;
      if (planner_)
        ok = planner_->step(now, latest_pose_.p, cmd);
      if (!ok)
      {
        // 规划失败 → 原地悬停（实机这里应叠加飞控侧急停/悬停）
        if (!emitHoldSetpoint(now, pos_enu_, out)) { fail("nonfinite control target"); break; }
      }
      else if (!emitSetpoint(now, cmd, out)) { fail("nonfinite control target"); break; }

      // 区分 EGO 轨迹指令与巡航阶段的保底悬停，供运行日志核对输出来源。
      if (ok && cmd.trajectory_flag == ego_planner::TRAJECTORY_STATUS_READY)
        ++planner_setpoints_;
      else
        ++cruise_hold_setpoints_;

      // 航点到达由本地位置判定；推进本地航段并更新规划目标。
      if (leg_ + 1 < (int)route_enu_.size() && have_pos_)
      {
        if ((latest_pose_.p - route_enu_[leg_ + 1]).norm() < cfg_.wp_reach_tol)
        {
          leg_++;
          if (leg_ + 1 < (int)route_enu_.size())
          {
            if (planner_)
              planner_->setLeg(route_enu_[leg_], route_enu_[leg_ + 1], leg_ + 2 == (int)route_enu_.size());
          }
          else
          {
            // 最后航点已返回 KML 起点：保持巡航高度，等地面站降落。
            hold_pos_ = latest_pose_.p;
            hold_reason_ = HoldReason::MISSION_DONE;
            t_hold_clear_ = -1e9;
            phase_ = Phase::HOLD;
            t_phase_ = now;
          }
        }
      }
      break;
    }

    // S5b 安全悬停（任何飞行任务异常，仍在 Offboard 内）
    case Phase::HOLD:
    {
      // 这三类异常没法靠悬停解决，只能报错
      if (!health.fc_alive)
      {
        fail("fcu link lost during hold");
        break;
      }
      if (main_mode_ != MODE_OFFBOARD)
      {
        fail("dropped out of offboard during hold");
        break;
      }
      if (!have_pos_)
      {
        fail("no position estimate during hold");
        break;
      }

      // 原地悬停：冻结进入悬停那一刻的位置和航向。
      if (!emitHoldSetpoint(now, hold_pos_, out)) { fail("nonfinite control target"); break; }
      // 异常全部清除并稳定 -> 丢掉过期轨迹，回到巡航
      if (hold_reason_ != HoldReason::MISSION_DONE && health.all_clear &&
          now - t_hold_clear_ >= cfg_.hold_recover_confirm_s)
      {
        if (planner_)
          planner_->dropTrajectory();
        hold_reason_ = HoldReason::NONE;
        phase_ = Phase::OFFBOARD_CRUISE;
        t_phase_ = now;
      }
      break;
    }

    default:
      break;
    }
  }

} // namespace ego_onboard
