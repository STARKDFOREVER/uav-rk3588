// 板载主程序：装配 OffboardPilot / AsyncPlanner / LivoxSource / SerialPort，
// 并跑「取串口字节 -> 喂雷达帧 -> 推进规划 -> pilot.tick 发帧」的单线程主循环。
#include <onboard/async_planner.h>
#include <onboard/config_file.h>
#include <onboard/error_log.h>
#include <onboard/kml_planner_start.h>
#include <onboard/livox_source.h>
#include <onboard/record_writer.h>
#include <onboard/route_utils.h>
#include "serial_port.h"
#include <csignal>
#include <cstdio>
#include <memory>
#include <sys/resource.h>
using namespace ego_onboard;
using namespace ego_mav;
namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int) { stopped=1; }
void usage() {
  std::puts("onboard_main --enable-control --serial <device> --config <yaml> --kml <path>\n"
    "             --livox-config <sdk.json> --lidar-ip <ip> [--baud 921600]\n"
    "             [--record-dir <directory>] [--duration <seconds>] [--error-dir <directory>]\n"
    "             (runtime errors go to <directory>/onboard_error.log)\n"
    "Record directories include handover.csv and raw fcu_rx.bin/fcu_tx.bin MAVLink streams with chunk indexes.\n"
    "Waits for the local KML at the configured path, then starts automatically.\n"
    "No arm/takeoff commands; GCS starts flight. --duration begins after KML loading.\n"
    "No synthetic cloud is permitted.");
}
}
int main(int argc,char **argv) {
  // 错误只写文件、不打印；先给默认目录，保证参数解析失败也能落盘。
  setErrorLogDir("error");
  // 只保留实机飞行：--enable-control 必须显式给出，避免误启动。
  bool control=false,kml_override=false;
  std::string dev,config,kml,lidar_config,lidar_ip,record,error_dir;
  int baud=921600; double duration=0;
  try {
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i];
      auto value=[&]() -> std::string { if(i+1>=argc) throw std::runtime_error("missing value for "+a); return argv[++i]; };
      if(a=="--serial") dev=value();
      else if(a=="--config") config=value();
      else if(a=="--kml") {kml=value();kml_override=true;}
      else if(a=="--livox-config") lidar_config=value();
      else if(a=="--lidar-ip") lidar_ip=value();
      else if(a=="--record-dir") record=value();
      else if(a=="--error-dir") error_dir=value();
      else if(a=="--baud") baud=std::stoi(value());
      else if(a=="--duration") duration=std::stod(value());
      else if(a=="--enable-control") control=true;
      else if(a=="--help" || a=="-h") {usage();return 0;}
      else throw std::runtime_error("unknown option: "+a);
    }
    if(!error_dir.empty()) setErrorLogDir(error_dir);
    // 第三方库（EGO/A*/SDK）的错误也写入同一文件，控制台不再显示。
    redirectStderrToErrorLog();
    if(!std::isfinite(duration)||duration<0) throw std::runtime_error("duration must be finite and nonnegative");
    if(!control) throw std::runtime_error("--enable-control is required: only real-aircraft control is supported");
    if(dev.empty()) throw std::runtime_error("--serial is required");
    if(config.empty()) throw std::runtime_error("--config is required to preserve deployment parameters");
    if(lidar_config.empty()||lidar_ip.empty()) throw std::runtime_error("--livox-config and --lidar-ip are required");
    AppConfig app; std::string error;
    if(!loadConfigFile(config,app,error)) throw std::runtime_error(error);
    if(kml_override) app.mission_input.file=kml;
    if(app.mission_input.file.empty()) throw std::runtime_error("mission_input.file is required");
    std::vector<GeoWaypoint> route;
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop); std::signal(SIGPIPE,SIG_IGN);
    // Always validate the local KML before opening any device.
    MissionInput input; input.start(app.mission_input);
    if(app.mission_input.wait_for_file) {
      std::printf("[STATE] WAIT_KML path=%s; waiting for a local task\n",app.mission_input.file.c_str());
      std::fflush(stdout);
    }
    std::string last_error;
    // 轮询直到任务就绪（READY）或失败（FAILED）；期间不打开任何设备。
    while(input.state()!=MissionInputState::READY && input.state()!=MissionInputState::FAILED && !stopped) {
      input.step();
      if(input.error()!=last_error) {
        last_error=input.error();
        if(!last_error.empty()) {std::printf("[KML] %s\n",last_error.c_str());std::fflush(stdout);}
      }
      if(input.state()==MissionInputState::WAIT_KML) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if(stopped) {std::puts("[STOP] KML wait cancelled; no device opened");return 0;}
    if(input.state()==MissionInputState::FAILED) throw std::runtime_error(input.error());
    route=input.mission().route;
    std::puts("[KML READY] valid task accepted; starting flight control automatically");
    // 解析阶段已补齐返航段，这里只同步巡航高度并打印任务规模。
    app.pilot.cruise_alt_m=route.front().alt_rel_m;
    std::printf("[KML] %zu local route points, height %.2fm\n",route.size(),route.front().alt_rel_m);
    // 实机控制硬门槛：RTK 固定解（fix=6）、GPS/感知失效保护、EGO 规划器缺一不可。
    if(app.pilot.gps_min_fix<6||!app.pilot.gps_failsafe||!app.pilot.sensing_failsafe)
      throw std::runtime_error("physical RTK control requires min_fix=6 and GPS/sensing failsafes");
    app.adapter.strict_pose_coverage=true;
    // 记录/诊断文件统一由 RecordWriter 管理；record 为空则不记录。
    // 录像是事后分析，打不开只告警，不影响飞行。
    RecordWriter rec;
    std::string record_error;
    if(!rec.open(record,record_error))
      std::printf("[RECORD] %s; continuing without record files\n",record_error.c_str());
    std::unique_ptr<AsyncPlanner> async(new AsyncPlanner(app.adapter,app.pilot.cruise_alt_m));
    PlannerIO *planner=async.get();
    OffboardPilot pilot; pilot.setConfig(app.pilot);pilot.setPlanner(planner);
    SerialPort serial; LivoxSource lidar;
    if(!serial.open(dev,baud,error)) throw std::runtime_error(error);
    if(!lidar.start(lidar_config,lidar_ip,error,&app.lidar)) throw std::runtime_error(error);
    pilot.start(route);
    std::puts("[PLANNER] valid KML received: planner enabled; waiting for valid inputs");
    std::puts("[HARDWARE] real FCU; SDK2 cloud only");
    std::printf("[LIDAR PROFILE] policy=%s SN=%s type=%d frame=%.3fs range=[%.2f,%.2f] queue=%zu\n",app.lidar.allow_zero_crc?"mid360s_sample":"strict",app.lidar.expected_serial.c_str(),app.lidar.point_data_type,app.lidar.frame_duration_s,app.lidar.min_range_m,app.lidar.max_range_m,app.lidar.queue_capacity);
    std::puts("[CONTROL] GCS owns arm/takeoff/landing; onboard follows its local KML route");
    const double begin=monotonicSeconds(); double now=begin,previous=now,last_report=-1e9,last_handover_log=-1e9;
    // 丢星后的推算位置单独落盘（相对项目根，与 start_onboard.sh 的 cwd 一致）；打不开只提示，不影响飞行。
    std::string gps_error;
    if(!rec.openGpsLoss("gps_loss_location",gps_error))
      std::printf("[GPS LOSS] %s; post-loss positions will not be saved\n",gps_error.c_str());
    bool ptp_ready=false;
    double max_loop=0,last_input_ok=-1e9,clock_stable_since=begin,alignment_bad_since=-1e9; uint64_t aligned=0,rejected=0;
    bool planner_bound=false;
    Phase last_phase=pilot.phase();
    EpochBridge epoch; epoch.sample(monotonicSeconds(),unixSeconds());
    uint64_t sync_generation=0; bool failed=false;
    while(!stopped && (duration==0 || (now-begin)<duration)) {
      now=monotonicSeconds();
      const double gap=now-previous;previous=now;max_loop=std::max(max_loop,gap);
      // 单拍超过 150ms 就停机：setpoint 一旦断流，飞控会自行退出 Offboard。
      if(gap>0.15) {pilot.stop("control loop deadline exceeded");failed=true;break;}
      pilot.setReceiveTime(now);
      if(!serial.isOpen()) {pilot.stop("serial disconnected");failed=true;break;}
      std::vector<uint8_t> bytes;serial.take(bytes);
      if(!bytes.empty()) {
        rec.rawRx(now,bytes.data(),bytes.size());
        pilot.onMavlinkBytes(bytes.data(),bytes.size());
      }
      const bool stable=epoch.sample(now,unixSeconds());
      // 位姿/校时判据只读最近回调时间戳，本拍算一次后供帧接纳、预规划与日志共用。
      const bool pose_ok=pilot.poseFresh(now),clock_ok=pilot.clockReady(now);
      // 已进入实际飞行阶段：时间边界一旦破坏，setpoint 会失去意义。
      const bool active=pilot.phase()==Phase::PRE_OFFBOARD||pilot.phase()==Phase::OFFBOARD_CRUISE||pilot.phase()==Phase::HOLD;
      // 时间边界被破坏（系统时钟 step / 飞控校时重置）：清时间相关运行态并重新计时。
      auto resetTimeState=[&]{ clock_stable_since=now;last_input_ok=-1e9;planner->resetClockState();alignment_bad_since=now; };
      if(!stable) {
        resetTimeState();
        if(active) { pilot.stop("system clock step");failed=true;break; }
      }
      else if(sync_generation!=pilot.timeAlignment().generation()) {
        // 飞控校时被重置：映射需要重新收敛，不立即终止任务。
        sync_generation=pilot.timeAlignment().generation(); resetTimeState();
      }
      // 校时连续失效超过 1 s 才终止控制，给重新收敛留窗口。
      if(!clock_ok) {
        if(alignment_bad_since<0) alignment_bad_since=now;
        if(active&&now-alignment_bad_since>1.0) { pilot.stop("clock alignment lost");failed=true;break; }
      } else alignment_bad_since=-1e9;
      const auto lidar_status=lidar.stats();
      const double last_sync=epoch.toMonotonic(lidar_status.last_sync_ns*1e-9);
      // PTP 就绪 = 雷达同步类型为 PTP、状态报文新鲜、最近同步时刻落在 [-5ms, 2s) 窗口内。
      ptp_ready=lidar_status.sync_type==1 && now-lidar_status.state_rx<2.0 &&
                now-last_sync>=-0.005 && now-last_sync<2.0;
      ego_sensor::LidarFrame frame;
      while(lidar.pop(frame)) {
        const double raw=frame.t_base;
        frame.t_base=epoch.toMonotonic(raw);
        const double end=frame.points.empty()?frame.t_base:frame.t_base+frame.points.back().t_rel;
        // 一帧点云被接纳需同时满足：PTP 稳定 ≥2s、雷达帧带 PTP 时间、飞控校时与位姿新鲜、
        // 帧尾时间早于当前且延迟在 sensing_timeout 内（防止补发的旧帧进图）。
        const bool timing_ready=ptp_ready&&stable&&now-clock_stable_since>=2&&frame.time_type==1&&!frame.points.empty()&&
            clock_ok&&pose_ok&&now-end<app.adapter.sensing_timeout_s&&end<=now+0.005;
        // Missing CRC and configured packet acceptance are separate properties.
        const bool crc_verified=frame.payload_crc_present;
        const bool packet_policy_ok=crc_verified||app.lidar.allow_zero_crc;
        const bool ok=packet_policy_ok&&timing_ready;
        rec.scan(raw,frame.t_base,now,int(frame.time_type),frame.points.size(),end-frame.t_base,ok,timing_ready,crc_verified,packet_policy_ok);
        if(ok) {planner->onScanFrame(frame);last_input_ok=now;++aligned;} else ++rejected;
      }
      pilot.setInputReady(ptp_ready&&now-last_input_ok<app.adapter.sensing_timeout_s&&now-lidar_status.last_rx<app.adapter.sensing_timeout_s);
      if(!planner_bound&&pilot.origin().valid)
        planner_bound=bindKmlPlanner(route,pilot.origin(),*planner);
      // 接管前预规划：KML 就绪即开始计算，但预览轨迹不外发；真正切模式时会丢弃重算。
      const bool inputs_ready=pose_ok&&clock_ok&&pilot.gpsFixType()>=app.pilot.gps_min_fix&&planner->sensingHealthy();
      preplanKml(planner_bound,inputs_ready,pilot.phase(),now,pilot.latestPose().p,*planner);
      std::vector<std::vector<uint8_t>> tx; pilot.tick(now,tx);
      for(const auto &f:tx) {
        if(!serial.write(f.data(),f.size())) {pilot.stop("serial write failed or timed out");failed=true;break;}
        rec.rawTx(now,f.data(),f.size());
      }
      if(pilot.phase()!=last_phase) {
        // Preview may have aged while GCS starts flight: never reuse it at takeover.
        // 预览轨迹可能在地面站起飞期间已过期，切 Offboard 前一律丢弃。
        if(pilot.phase()==Phase::PRE_OFFBOARD)planner->dropTrajectory();
        // 失败状态只写入错误日志，不打印具体原因。
        if(pilot.phase()==Phase::FAILED) {
          char line[96]={};
          std::snprintf(line,sizeof(line),"[STATE] %.2fs FAILED",now-begin);
          logError(line);
        }
        else std::printf("[STATE] %.2fs %s (%s)\n",now-begin,phaseName(pilot.phase()),holdReasonName(pilot.holdReason()));
        last_phase=pilot.phase();}
      const bool log_handover=rec.recording()&&now-last_handover_log>=0.2;
      const bool report_tick=now-last_report>=1;
      // 同一拍的 handover 诊断只算一次，落盘与打印共用同一份快照。
      HandoverDiagnostics hd;
      if(log_handover||report_tick) hd=pilot.handoverDiagnostics(now);
      if(log_handover) {
        last_handover_log=now;
        rec.handover(now-begin,pilot.phase(),hd);
      }
      if(report_tick) {
        last_report=now; const auto st=async->stats(); const auto &ls=lidar_status; const auto &ts=pilot.timeAlignment();
        const uint64_t crc=pilot.crcErrors();
        rec.flush();
        std::printf("[STATUS] t=%.1f %s GPS=%u pose=%d sync=%d n=%d RTT=%.2fms reject=%llu lidar=%llu type=%u ptp=%d aligned=%llu map=%d loop_max=%.1fms CRC=%llu\n",now-begin,phaseName(pilot.phase()),pilot.gpsFixType(),pose_ok,clock_ok,ts.samples(),ts.rtt()*1000,(unsigned long long)ts.rejected(),(unsigned long long)ls.packets,ls.time_type,ptp_ready,(unsigned long long)aligned,st.inflate_samples,max_loop*1000,(unsigned long long)crc);
        std::printf("[HANDOVER] phase=%s ready=%d stable=%.2fs fc=%d armed=%d mode=%u/%u pose=%d timing=%d sensing=%d gps=%d route=%d xy=%.2fm/%d alt=%.2fm/%d speed=%.2fmps/%d ack=%d/%u age=%.2fs\n",
          phaseName(pilot.phase()),hd.ready,hd.ready_stable_s,hd.fc_alive,hd.armed,unsigned(hd.main_mode),unsigned(hd.sub_mode),
          hd.pose_ok,hd.timing_ok,hd.sensing_ok,hd.gps_ok,hd.route_valid,hd.horizontal_error_m,hd.horizontal_position_ok,
          hd.altitude_rel_m,hd.altitude_ok,hd.speed_mps,hd.speed_ok,hd.mode_ack_seen,unsigned(hd.mode_ack_result),hd.mode_ack_age_s);
        rusage memory{}; getrusage(RUSAGE_SELF, &memory);
        std::printf("[EGO] trajectory=%d plans_ok=%llu plans_fail=%d optimizations=%llu astar_ok=%llu/%llu tracked=%llu holds=%llu sent=%llu cruise_holds=%llu plan_last=%.1fms plan_max=%.1fms rss_peak=%.1fMiB\n",
          st.trajectory_active,(unsigned long long)st.plan_success,st.plan_fail,
          (unsigned long long)st.ego_optimizations,(unsigned long long)st.astar_successes,
          (unsigned long long)st.astar_searches,(unsigned long long)st.tracking_commands,
          (unsigned long long)st.hold_commands,(unsigned long long)pilot.plannerSetpoints(),
          (unsigned long long)pilot.cruiseHoldSetpoints(),st.last_plan_ms,st.plan_ms_max,memory.ru_maxrss/1024.0);
        rec.timing({now-begin,phaseName(pilot.phase()),int(pilot.gpsFixType()),clock_ok,(uint64_t)ts.samples(),ts.rtt()*1000,ts.residual()*1000,ls.packets,int(ls.time_type),aligned,rejected,st.inflate_samples,pose_ok,max_loop*1000,crc,ptp_ready,ls.time_offset_ns,ls.crc_missing,st.step_calls});

      }
      // GPS 丢星期间把推算位置写入本地文件，供事后分析（内部 1 Hz 限频）。
      if(rec.gpsLossDue(now)&&!pilot.gpsOk(now)) {
        GeoFix estimate;double age=0;
        if(pilot.gpsEstimate(estimate,age))
          rec.gpsLoss(now-begin,phaseName(pilot.phase()),pilot.gpsFixType(),estimate,age,pilot.latestPose().p);
      }
      if(failed||pilot.phase()==Phase::FAILED) {failed=true;break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto final_lidar=lidar.stats();
    lidar.stop(); serial.close();
    std::printf("[LIDAR] packets=%llu frames=%llu invalid=%llu queue_drops=%llu missing_crc=%llu (zero CRC means absent; packet acceptance follows configured policy)\n", (unsigned long long)final_lidar.packets,(unsigned long long)final_lidar.frames,(unsigned long long)final_lidar.invalid,(unsigned long long)final_lidar.queue_drops,(unsigned long long)final_lidar.crc_missing);
    const auto stats=async->stats();
    // 失败退出只写日志，不显示原因；正常结束仍打印阶段与原因便于诊断。
    if(pilot.phase()==Phase::FAILED) {
      char line[256]={};
      std::snprintf(line,sizeof(line),"[RESULT] FAILED maps=%d points=%ld plans=%d failures=%d aligned=%llu rejected=%llu max_loop=%.2fms",
        stats.inflate_samples,stats.cloud_points,stats.replan,stats.plan_fail,
        (unsigned long long)aligned,(unsigned long long)rejected,max_loop*1000);
      logError(line);
    }
    else
      std::printf("[RESULT] phase=%s reason=%s error=%s maps=%d points=%ld plans=%d failures=%d aligned=%llu rejected=%llu max_loop=%.2fms\n",phaseName(pilot.phase()),holdReasonName(pilot.holdReason()),pilot.lastError(),stats.inflate_samples,stats.cloud_points,stats.replan,stats.plan_fail,(unsigned long long)aligned,(unsigned long long)rejected,max_loop*1000);
    for(const auto &e:pilot.crcErrorsByMsgid()) {
      char line[96]={};
      std::snprintf(line,sizeof(line),"[CRC] msg=%u failures=%llu",e.first,(unsigned long long)e.second);
      logError(line);
    }
    return failed?1:0;
  } catch(const std::exception &e) { logError(std::string("[FATAL] ")+e.what());return 2; }
}
