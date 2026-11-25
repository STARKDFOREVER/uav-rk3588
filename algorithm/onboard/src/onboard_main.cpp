// 板载主程序：装配 OffboardPilot / AsyncPlanner / LivoxSource / SerialPort，
// 并跑「取串口字节 -> 喂雷达帧 -> 推进规划 -> pilot.tick 发帧」的单线程主循环。
#include <onboard/async_planner.h>
#include <onboard/config_file.h>
#include <onboard/error_log.h>
#include <onboard/kml_planner_start.h>
#include <onboard/livox_source.h>
#include <onboard/route_utils.h>
#include "serial_port.h"
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sys/stat.h>
#include <sys/resource.h>
using namespace ego_onboard;
using namespace ego_mav;
namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int) { stopped=1; }
void usage() {
  std::puts("onboard_main --lidar-only --config <yaml> --livox-config <sdk.json> --lidar-ip <ip> [--duration <seconds>]\n"
    "onboard_main --prepare-only --config <yaml> --kml <path>\n"
    "onboard_main --serial <device|tcp://host:port> --config <yaml> [--baud 921600]\n"
    "             [--livox-config <sdk.json> --lidar-ip <ip>] [--record-dir <directory>]\n"
    "             [--duration <seconds>] [--enable-control --kml <path>]\n"
    "Record directories include handover.csv and raw fcu_rx.bin/fcu_tx.bin MAVLink streams with chunk indexes.\n"
    "             [--no-wait-kml] (observe-only diagnostics; skip the default inbox)\n"
    "             [--error-dir <directory>] (runtime errors go to <directory>/onboard_error.log)\n"
    "Waits for the local KML at the configured path, then starts automatically.\n"
    "No arm/takeoff commands; GCS starts flight. --duration begins after KML loading.\n"
    "Physical mode defaults to observe-only. No synthetic cloud is permitted.\n"
    "Observe-only sends heartbeat, telemetry requests and clock sync only.");
}
// 观察模式下落盘一帧原始点云，坐标是雷达系且未去畸变，仅供排查同步/CRC 问题。
void snapshot(const std::string &path,const ego_sensor::LidarFrame &f) {
  std::ofstream out(path);
  out << "# Raw sensor coordinates; NOT deskewed\nVERSION .7\nFIELDS x y z intensity time\nSIZE 4 4 4 4 8\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nWIDTH " << f.points.size()
      << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << f.points.size() << "\nDATA ascii\n";
  for(const auto &p:f.points) out << p.p.x()<<' '<<p.p.y()<<' '<<p.p.z()<<' '<<p.reflectivity<<' '<<std::setprecision(12)<<p.t_rel<<'\n';
}
}
int main(int argc,char **argv) {
  // 错误只写文件、不打印；先给默认目录，保证参数解析失败也能落盘。
  setErrorLogDir("error");
  // control 默认 false = observe-only；--enable-control 才允许发 setpoint。
  bool prepare=false,control=false,lidar_only=false,no_wait_kml=false,kml_override=false;
  std::string dev,config,kml,lidar_config,lidar_ip,record,error_dir;
  int baud=921600; double duration=0;
  try {
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i];
      auto value=[&]() -> std::string { if(i+1>=argc) throw std::runtime_error("missing value for "+a); return argv[++i]; };
      if(a=="--lidar-only") lidar_only=true;
      else if(a=="--serial") dev=value();
      else if(a=="--config") config=value();
      else if(a=="--kml") {kml=value();kml_override=true;}
      else if(a=="--livox-config") lidar_config=value();
      else if(a=="--lidar-ip") lidar_ip=value();
      else if(a=="--record-dir") record=value();
      else if(a=="--error-dir") error_dir=value();
      else if(a=="--baud") baud=std::stoi(value());
      else if(a=="--duration") duration=std::stod(value());
      else if(a=="--prepare-only") prepare=true;
      else if(a=="--no-wait-kml") no_wait_kml=true;
      else if(a=="--enable-control") control=true;
      else if(a=="--observe-only") control=false;
      else if(a=="--help" || a=="-h") {usage();return 0;}
      else throw std::runtime_error("unknown option: "+a);
    }
    if(!error_dir.empty()) setErrorLogDir(error_dir);
    // 第三方库（EGO/A*/SDK）的错误也写入同一文件，控制台不再显示。
    redirectStderrToErrorLog();
    if(!std::isfinite(duration)||duration<0) throw std::runtime_error("duration must be finite and nonnegative");
    if(!prepare&&!lidar_only&&dev.empty()) throw std::runtime_error("--serial is required unless using --prepare-only or --lidar-only");
    if(lidar_only&&(prepare||control||!dev.empty()||lidar_config.empty()||lidar_ip.empty())) throw std::runtime_error("lidar-only requires SDK config and lidar IP, without serial/prepare/control");
    if(config.empty()) throw std::runtime_error("--config is required to preserve deployment parameters");
    if(no_wait_kml&&control) throw std::runtime_error("--no-wait-kml is only for observe-only diagnostics");
    if(lidar_only&&kml_override) throw std::runtime_error("--lidar-only does not load a KML");
    AppConfig app; std::string error;
    if(!loadConfigFile(config,app,error)) throw std::runtime_error(error);
    if(kml_override) app.mission_input.file=kml;
    if(prepare||no_wait_kml) app.mission_input.wait_for_file=false;
    if(lidar_only||(no_wait_kml&&!kml_override)) app.mission_input.file.clear();
    std::vector<GeoWaypoint> route;
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop); std::signal(SIGPIPE,SIG_IGN);
    // Always validate a supplied KML before opening any device.
    if(!app.mission_input.file.empty()) {
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
      std::puts("[KML READY] valid task accepted; starting configured mode automatically");
    }
    else if(control||prepare) throw std::runtime_error("physical control requires an explicit local KML");
    if(!route.empty()) {
      // 解析阶段已补齐返航段，这里只同步巡航高度并打印任务规模。
      app.pilot.cruise_alt_m=route.front().alt_rel_m;
      std::printf("[KML] %zu local route points, height %.2fm\n",route.size(),route.front().alt_rel_m);
    }
    const bool kml_ready=route.size()>=2;
    if(prepare) { std::puts("[READY] KML validated; no device opened"); return 0; }
    if(control&&(lidar_config.empty()||lidar_ip.empty()))
      throw std::runtime_error("control requires --livox-config and --lidar-ip");
    // 实机控制硬门槛：RTK 固定解（fix=6）、GPS/感知失效保护、EGO 规划器缺一不可。
    if(control&&(app.pilot.gps_min_fix<6||!app.pilot.gps_failsafe||!app.pilot.sensing_failsafe||!app.pilot.use_planner))
      throw std::runtime_error("physical RTK control requires min_fix=6, GPS/sensing failsafes, and EGO planner");
    app.pilot.observe_only=!control;
    app.adapter.strict_pose_coverage=true;
    std::ofstream diag,frame_log,handover_log,raw_rx,raw_tx,rx_chunks,tx_chunks;
    uint64_t rx_offset=0,tx_offset=0;
    if(!record.empty()) {
      if(::mkdir(record.c_str(),0755)!=0&&errno!=EEXIST) throw std::runtime_error("cannot create record directory");
      if(!lidar_only) {
        raw_rx.open(record+"/fcu_rx.bin",std::ios::binary);
        raw_tx.open(record+"/fcu_tx.bin",std::ios::binary);
        rx_chunks.open(record+"/fcu_rx_chunks.csv");
        tx_chunks.open(record+"/fcu_tx_chunks.csv");
      }
      if(!lidar_only&&(!raw_rx||!raw_tx||!rx_chunks||!tx_chunks))
        throw std::runtime_error("cannot open raw MAVLink diagnostic logs");
      diag.open(record+"/timing.csv"); frame_log.open(record+"/scans.csv");
      handover_log.open(record+"/handover.csv");
      if(!diag||!frame_log||!handover_log) throw std::runtime_error("cannot open diagnostic logs");
      // 落盘表头：timing.csv 每拍一行；scans.csv 每帧雷达；handover.csv 以 5Hz 采样接管判据。
      diag<<"elapsed,phase,gps_fix,clock_ok,sync_samples,rtt_ms,residual_ms,lidar_packets,time_type,aligned_frames,rejected_frames,map_frames,pose_fresh,max_loop_ms,crc_errors,ptp_ready,lidar_sync_offset_ns,lidar_crc_missing,kml_ready,planner_steps\n";
      frame_log<<"raw_epoch,host_monotonic,receive_monotonic,time_type,points,span_s,aligned,timing_ready,crc_verified,packet_policy_ok\n";
      handover_log<<"elapsed,phase,phase_elapsed_s,fc_alive,armed,main_mode,sub_mode,pose_ok,timing_ok,sensing_ok,gps_rx,gps_fix,gps_ok,route_valid,horizontal_error_m,horizontal_position_ok,altitude_rel_m,altitude_ok,speed_mps,speed_ok,ready,ready_stable_s,mode_switch_sent,waiting_offboard,mode_ack_seen,mode_ack_result,mode_ack_age_s\n";
      if(!lidar_only) {
        rx_chunks<<"host_monotonic,offset,size\n";
        tx_chunks<<"host_monotonic,offset,size\n";
      }
    }
    std::unique_ptr<AsyncPlanner> async(new AsyncPlanner(app.adapter,app.pilot.cruise_alt_m));
    PlannerIO *planner=async.get();
    OffboardPilot pilot; pilot.setConfig(app.pilot);pilot.setPlanner(planner);
    SerialPort serial; LivoxSource lidar;
    if(!lidar_only&&!serial.open(dev,baud,error)) throw std::runtime_error(error);
    if(!lidar_config.empty()&&!lidar.start(lidar_config,lidar_ip,error,&app.lidar)) throw std::runtime_error(error);
    if(app.pilot.observe_only) pilot.startObserving(); else pilot.start(route);
    if(kml_ready) std::puts("[PLANNER] valid KML received: planner enabled; waiting for valid inputs");
    std::puts(lidar_only?"[LIDAR ONLY] Ethernet SDK2 directly to onboard; FCU serial is not opened":"[HARDWARE] real FCU; SDK2 cloud only");
    std::printf("[LIDAR PROFILE] policy=%s SN=%s type=%d frame=%.3fs range=[%.2f,%.2f] queue=%zu\n",app.lidar.allow_zero_crc?"mid360s_sample":"strict",app.lidar.expected_serial.c_str(),app.lidar.point_data_type,app.lidar.frame_duration_s,app.lidar.min_range_m,app.lidar.max_range_m,app.lidar.queue_capacity);
    std::puts(app.pilot.observe_only?"[OBSERVE] no task/mode/motion commands":"[CONTROL] GCS owns arm/takeoff/landing; onboard follows its local KML route");
    const double begin=monotonicSeconds(); double now=begin,previous=now,last_report=-1e9,last_handover_log=-1e9;
    // 丢星后的推算位置单独落盘（相对项目根，与 start_onboard.sh 的 cwd 一致）。
    const std::string gps_loss_dir="gps_loss_location";
    std::ofstream gps_loss_log;
    if(::mkdir(gps_loss_dir.c_str(),0755)!=0&&errno!=EEXIST)
      std::printf("[GPS LOSS] cannot create %s; post-loss positions will not be saved\n",gps_loss_dir.c_str());
    else {
      const std::string path=gps_loss_dir+"/gps_loss.csv";
      struct stat existing{};
      const bool fresh=::stat(path.c_str(),&existing)!=0||existing.st_size==0;
      gps_loss_log.open(path,std::ios::app);
      if(!gps_loss_log) std::printf("[GPS LOSS] cannot open %s\n",path.c_str());
      else if(fresh) gps_loss_log<<"unix_time,elapsed,phase,fix,lat,lon,alt,age,pos_x,pos_y,pos_z\n";
    }
    double last_gps_loss_log=-1e9;
    bool ptp_ready=false;
    double max_loop=0,last_input_ok=-1e9,clock_stable_since=begin; uint64_t aligned=0,rejected=0;
    bool planner_bound=false;
    bool snapshot_done=false; Phase last_phase=pilot.phase();
    EpochBridge epoch; epoch.sample(monotonicSeconds(),unixSeconds());
    uint64_t sync_generation=0; bool failed=false;
    while(!stopped && (duration==0 || (now-begin)<duration)) {
      now=monotonicSeconds();
      const double gap=now-previous;previous=now;max_loop=std::max(max_loop,gap);
      // 控制模式下单拍超过 150ms 就停机：setpoint 一旦断流，飞控会自行退出 Offboard。
      if(control&&gap>0.15) {pilot.stop("control loop deadline exceeded");failed=true;break;}
      pilot.setReceiveTime(now);
      if(!lidar_only&&!serial.isOpen()) {pilot.stop("serial disconnected");failed=true;break;}
      std::vector<uint8_t> bytes;serial.take(bytes);
      if(!bytes.empty()) {
        if(raw_rx) {
          raw_rx.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
          rx_chunks<<std::setprecision(16)<<now<<','<<rx_offset<<','<<bytes.size()<<'\n';
          rx_offset+=bytes.size();
        }
        pilot.onMavlinkBytes(bytes.data(),bytes.size());
      }
      const bool stable=epoch.sample(monotonicSeconds(),unixSeconds());
      if(!stable || sync_generation!=pilot.timeAlignment().generation()) {
        clock_stable_since=now;last_input_ok=-1e9;sync_generation=pilot.timeAlignment().generation();
        // Clock discontinuity: discard pose history and in-flight frames only.
        // The occupancy map is world-frame and stays valid, so no rebuild here.
        const bool active=pilot.phase()==Phase::PRE_OFFBOARD||pilot.phase()==Phase::OFFBOARD_CRUISE||pilot.phase()==Phase::HOLD;
        if(control&&active) { pilot.stop("clock discontinuity: restart perception before control");failed=true;break; }
        if(planner) planner->resetClockState();
      }
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
            pilot.clockReady(now)&&pilot.poseFresh(now)&&now-end<app.adapter.sensing_timeout_s&&end<=now+0.005;
        // Missing CRC and configured packet acceptance are separate properties.
        const bool crc_verified=frame.payload_crc_present;
        const bool packet_policy_ok=crc_verified||app.lidar.allow_zero_crc;
        const bool ok=!lidar_only&&packet_policy_ok&&timing_ready;
        if(frame_log) frame_log<<std::setprecision(16)<<raw<<','<<frame.t_base<<','<<now<<','<<int(frame.time_type)<<','<<frame.points.size()<<','<<end-frame.t_base<<','<<ok<<','<<timing_ready<<','<<crc_verified<<','<<packet_policy_ok<<'\n';
        if(!control&&!snapshot_done&&!record.empty()) {snapshot(record+"/raw_cloud.pcd",frame);snapshot_done=true;}
        if(ok) {planner->onScanFrame(frame);last_input_ok=now;++aligned;} else ++rejected;
      }
      pilot.setInputReady(ptp_ready&&now-last_input_ok<app.adapter.sensing_timeout_s&&now-lidar.stats().last_rx<app.adapter.sensing_timeout_s);
      if(kml_ready&&!planner_bound&&planner&&pilot.origin().valid)
        planner_bound=bindKmlPlanner(route,pilot.origin(),*planner);
      // A valid KML begins onboard planning before arm/takeover. Preview is not sent.
      // 接管前预规划：KML 就绪即开始计算，但预览轨迹不外发；真正切模式时会丢弃重算。
      if(!lidar_only&&planner) {
        const bool inputs_ready=pilot.poseFresh(now)&&pilot.clockReady(now)&&
          pilot.gpsFixType()>=app.pilot.gps_min_fix&&planner->sensingHealthy();
        preplanKml(kml_ready,planner_bound,inputs_ready,pilot.phase(),now,pilot.latestPose().p,*planner);
      }
      std::vector<std::vector<uint8_t>> tx; if(!lidar_only) pilot.tick(now,tx);
      for(const auto &f:tx) {
        if(!serial.write(f.data(),f.size())) {pilot.stop("serial write failed or timed out");failed=true;break;}
        if(raw_tx) {
          raw_tx.write(reinterpret_cast<const char*>(f.data()),f.size());
          tx_chunks<<std::setprecision(16)<<now<<','<<tx_offset<<','<<f.size()<<'\n';
          tx_offset+=f.size();
        }
      }
      if(pilot.phase()!=last_phase) {
        // Preview may have aged while GCS starts flight: never reuse it at takeover.
        // 预览轨迹可能在地面站起飞期间已过期，切 Offboard 前一律丢弃。
        if(planner&&pilot.phase()==Phase::PRE_OFFBOARD)planner->dropTrajectory();
        // 失败状态只写入错误日志，不打印具体原因。
        if(pilot.phase()==Phase::FAILED) {
          char line[96]={};
          std::snprintf(line,sizeof(line),"[STATE] %.2fs FAILED",now-begin);
          logError(line);
        }
        else std::printf("[STATE] %.2fs %s (%s)\n",now-begin,phaseName(pilot.phase()),holdReasonName(pilot.holdReason()));
        last_phase=pilot.phase();}
      if(handover_log && now-last_handover_log>=0.2) {
        last_handover_log=now;
        const auto d=pilot.handoverDiagnostics(now);
        handover_log<<std::setprecision(12)<<now-begin<<','<<phaseName(pilot.phase())<<','<<d.phase_elapsed_s<<','
          <<d.fc_alive<<','<<d.armed<<','<<unsigned(d.main_mode)<<','<<unsigned(d.sub_mode)<<','
          <<d.pose_ok<<','<<d.timing_ok<<','<<d.sensing_ok<<','<<d.gps_rx<<','<<unsigned(d.gps_fix_type)<<','
          <<d.gps_ok<<','<<d.route_valid<<','<<d.horizontal_error_m<<','<<d.horizontal_position_ok<<','
          <<d.altitude_rel_m<<','<<d.altitude_ok<<','<<d.speed_mps<<','<<d.speed_ok<<','<<d.ready<<','
          <<d.ready_stable_s<<','<<d.mode_switch_sent<<','<<d.waiting_offboard<<','<<d.mode_ack_seen<<','
          <<unsigned(d.mode_ack_result)<<','<<d.mode_ack_age_s<<'\n';
        handover_log.flush();
      }
      if(now-last_report>=1) {
        last_report=now; const auto st=async->stats(); const auto ls=lidar.stats(); const auto &ts=pilot.timeAlignment();
        if(raw_rx) { raw_rx.flush(); raw_tx.flush(); rx_chunks.flush(); tx_chunks.flush(); }
        std::printf("[STATUS] t=%.1f %s GPS=%u pose=%d sync=%d n=%d RTT=%.2fms reject=%llu lidar=%llu type=%u ptp=%d aligned=%llu map=%d loop_max=%.1fms CRC=%llu\n",now-begin,phaseName(pilot.phase()),pilot.gpsFixType(),pilot.poseFresh(now),pilot.clockReady(now),ts.samples(),ts.rtt()*1000,(unsigned long long)ts.rejected(),(unsigned long long)ls.packets,ls.time_type,ptp_ready,(unsigned long long)aligned,st.inflate_samples,max_loop*1000,(unsigned long long)pilot.crcErrors());
        const auto d=pilot.handoverDiagnostics(now);
        std::printf("[HANDOVER] phase=%s fc=%d armed=%d mode=%u sub=%u pose=%d timing=%d sensing=%d gps_rx=%d fix=%u gps=%d route=%d xy=%.2fm/%d alt=%.2fm/%d speed=%.2fmps/%d ready=%d stable=%.2fs request=%d waiting=%d ack=%d result=%u age=%.2fs\n",
          phaseName(pilot.phase()),d.fc_alive,d.armed,unsigned(d.main_mode),unsigned(d.sub_mode),d.pose_ok,d.timing_ok,d.sensing_ok,
          d.gps_rx,unsigned(d.gps_fix_type),d.gps_ok,d.route_valid,d.horizontal_error_m,d.horizontal_position_ok,
          d.altitude_rel_m,d.altitude_ok,d.speed_mps,d.speed_ok,d.ready,d.ready_stable_s,d.mode_switch_sent,
          d.waiting_offboard,d.mode_ack_seen,unsigned(d.mode_ack_result),d.mode_ack_age_s);
        rusage memory{}; getrusage(RUSAGE_SELF, &memory);
        std::printf("[EGO] trajectory=%d plans_ok=%llu plans_fail=%d optimizations=%llu astar_ok=%llu/%llu tracked=%llu holds=%llu sent=%llu cruise_holds=%llu plan_last=%.1fms plan_max=%.1fms rss_peak=%.1fMiB\n",
          st.trajectory_active,(unsigned long long)st.plan_success,st.plan_fail,
          (unsigned long long)st.ego_optimizations,(unsigned long long)st.astar_successes,
          (unsigned long long)st.astar_searches,(unsigned long long)st.tracking_commands,
          (unsigned long long)st.hold_commands,(unsigned long long)pilot.plannerSetpoints(),
          (unsigned long long)pilot.cruiseHoldSetpoints(),st.last_plan_ms,st.plan_ms_max,memory.ru_maxrss/1024.0);
        if(diag) {diag<<std::setprecision(12)<<(now-begin)<<','<<phaseName(pilot.phase())<<','<<int(pilot.gpsFixType())<<','<<pilot.clockReady(now)<<','<<ts.samples()<<','<<ts.rtt()*1000<<','<<ts.residual()*1000<<','<<ls.packets<<','<<int(ls.time_type)<<','<<aligned<<','<<rejected<<','<<st.inflate_samples<<','<<pilot.poseFresh(now)<<','<<max_loop*1000<<','<<pilot.crcErrors()<<','<<ptp_ready<<','<<ls.time_offset_ns<<','<<ls.crc_missing<<','<<kml_ready<<','<<st.step_calls<<'\n';diag.flush();}

      }
      // GPS 丢星期间把推算位置写入本地文件，供事后分析。
      if(gps_loss_log&&now-last_gps_loss_log>=1.0&&!pilot.gpsOk(now)) {
        GeoFix estimate;double age=0;
        if(pilot.gpsEstimate(estimate,age)) {
          last_gps_loss_log=now;
          const auto p=pilot.latestPose().p;
          gps_loss_log<<std::setprecision(12)<<unixSeconds()<<','<<(now-begin)<<','<<phaseName(pilot.phase())<<','
            <<unsigned(pilot.gpsFixType())<<','<<estimate.lat_deg<<','<<estimate.lon_deg<<','<<estimate.alt_m<<','
            <<age<<','<<p.x()<<','<<p.y()<<','<<p.z()<<'\n';
          gps_loss_log.flush();
        }
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
    if(lidar_only&&final_lidar.frames==0) logError("[LIDAR] no frames received");
    return (failed||(lidar_only&&final_lidar.frames==0))?1:0;
  } catch(const std::exception &e) { logError(std::string("[FATAL] ")+e.what());return 2; }
}
