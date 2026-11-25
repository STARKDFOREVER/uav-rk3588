// Livox MID-360S 数据源实现。
// 数据流：SDK2 回调（points）-> 设备/包类型校验 -> FrameAssembler.addPacket
//         -> popFrame 得到整帧 -> 有界 queue_ -> 主循环 pop()。
// 设备锁定：只有 info 回调里序列号与 IP 都匹配、且句柄稳定后才 selected_，
// 此前所有点云包一律丢弃，避免多雷达环境下跌到错误设备。
// 帧异常处理：包间缺口超过 gap_factor 倍帧时长时，组装器会丢弃半帧并计入 invalid。
#include <onboard/livox_source.h>
#include <onboard/error_log.h>
#include <onboard/time_alignment.h>
#include <cstdio>
#include <rapidjson/document.h>
#include <data_handler/data_handler.h>
namespace ego_onboard {
LivoxSource::~LivoxSource() { stop(); }
// 启动：先落定过滤/组装参数，再初始化 SDK 并注册回调。
// 注意 setup 顺序——参数必须在 SDK 启动前设好，回调注册可在启动后立即进行。
bool LivoxSource::start(const std::string &config, const std::string &ip, std::string &error, const Config *settings) {
  config_ = settings ? *settings : Config();
  // 豁免 CRC 时必须能唯一识别设备（IP + 序列号）且格式固定，否则拒绝启动
  if (config_.allow_zero_crc && (ip.empty() || config_.expected_serial.empty() || config_.point_data_type != 1)) {
    error="Mid360S sample CRC profile requires an explicit IP, serial number and Cartesian High data"; return false;
  }
  // 重建组装器，避免沿用上次运行残留的半帧状态
  assembler_ = ego_sensor::FrameAssembler();
  ego_sensor::FrameAssembler::Config frame; frame.frame_duration=config_.frame_duration_s; frame.gap_factor=config_.gap_factor;
  assembler_.setConfig(frame);
  // 量程过滤在解析层完成：超出 [min_range, max_range] 的点直接丢弃
  ego_sensor::LivoxParser::Config parser;parser.min_range=config_.min_range_m;parser.max_range=config_.max_range_m;
  assembler_.setParserConfig(parser);
  // 仅按实测豁免配置决定是否放行 CRC=0 的包
  livox::lidar::DataHandler::GetInstance().AllowZeroMid360sPointCrc(config_.allow_zero_crc);
  wanted_ip_ = ip;
  if (!LivoxLidarSdkInit(config.c_str())) {
    // SDK 初始化失败时立刻反初始化，避免半初始化状态影响后续重试
    LivoxLidarSdkUninit(); error = "SDK2 initialization failed: check host_ip/local interface/UDP ports"; return false;
  }
  started_ = true;
  SetLivoxLidarPointCloudCallBack(points, this);
  SetLivoxLidarInfoCallback(state, this);
  SetLivoxLidarInfoChangeCallback(info, this);
  return true;
}
// 停止：仅在真正启动过时反初始化，并复位全局 CRC 豁免开关，避免影响其它实例
void LivoxSource::stop() {
  if (started_) { LivoxLidarSdkUninit(); started_ = false; }
  livox::lidar::DataHandler::GetInstance().AllowZeroMid360sPointCrc(false);
}
// 异步控制（工作模式/点格式）的应答日志：只写错误日志，失败不重试，由上层日志判断
void LivoxSource::dataType(livox_status s, uint32_t h, LivoxLidarAsyncControlResponse *r, void *) {
  char line[128]={};
  std::snprintf(line,sizeof(line),"[Livox] Cartesian High handle=%u status=%d ret=%d",h,int(s),r?int(r->ret_code):-1);
  logError(line);
}
void LivoxSource::mode(livox_status s, uint32_t h, LivoxLidarAsyncControlResponse *r, void *) {
  char line[128]={};
  std::snprintf(line,sizeof(line),"[Livox] NORMAL handle=%u status=%d ret=%d",h,int(s),r?int(r->ret_code):-1);
  logError(line);
}
// 设备信息回调：承担"选设备"职责。
// 只接受 MID-360S；序列号/IP 不匹配则忽略；已锁定后若换了句柄也忽略，
// 防抖地保证一次运行只绑定一台雷达。锁定成功后设置工作模式与点云格式。
void LivoxSource::info(uint32_t h, const LivoxLidarInfo *i, void *ctx) {
  auto &self = *static_cast<LivoxSource *>(ctx);
  if (!i || i->dev_type != kLivoxLidarTypeMid360s) return;
  {
    std::lock_guard<std::mutex> lock(self.mutex_);
    if ((!self.config_.expected_serial.empty() && self.config_.expected_serial != i->sn) || (!self.wanted_ip_.empty() && self.wanted_ip_ != i->lidar_ip) || (self.selected_ && self.handle_ != h)) return;
    self.handle_ = h; self.selected_ = true;
    self.stats_.serial = i->sn; self.stats_.ip = i->lidar_ip;
  }
  std::printf("[Livox] Mid360S SN=%s IP=%s\n", i->sn, i->lidar_ip);
  SetLivoxLidarWorkMode(h, kLivoxLidarNormal, mode, ctx);
  SetLivoxLidarPclDataType(h, static_cast<LivoxLidarPointDataType>(self.config_.point_data_type), dataType, ctx);
}
// 状态回调：解析 JSON 中的时间同步信息，写入 stats_ 供上层判断 PTP 是否就绪。
// 仅处理已锁定句柄；解析失败/字段类型不符时直接忽略，不抛异常。
void LivoxSource::state(uint32_t h, uint8_t type, const char *json, void *ctx) {
  if(!json || type!=kLivoxLidarTypeMid360s) return;
  rapidjson::Document d; d.Parse(json);
  if(d.HasParseError()||!d.IsObject()) return;
  auto &self=*static_cast<LivoxSource*>(ctx);
  std::lock_guard<std::mutex> lock(self.mutex_);
  if(!self.selected_||self.handle_!=h) return;
  if(d.HasMember("time_sync_type")&&d["time_sync_type"].IsUint()) self.stats_.sync_type=d["time_sync_type"].GetUint();
  if(d.HasMember("last_sync_time")&&d["last_sync_time"].IsUint64()) {
    self.stats_.last_sync_ns=d["last_sync_time"].GetUint64(); self.stats_.state_rx=monotonicSeconds();
  }
  if(d.HasMember("time_offset")&&d["time_offset"].IsInt64()) self.stats_.time_offset_ns=d["time_offset"].GetInt64();
}
// 点云包回调：过滤后交给组装器，组装出整帧则推入有界队列。
// 全过程持锁，但只做 memcpy 级操作与一次 popFrame，避免阻塞 UDP 回调线程。
void LivoxSource::points(uint32_t h, uint8_t type, LivoxLidarEthernetPacket *p, void *ctx) {
  auto &self = *static_cast<LivoxSource *>(ctx);
  std::lock_guard<std::mutex> lock(self.mutex_);
  if (!self.selected_ || self.handle_ != h || type != kLivoxLidarTypeMid360s || !p) return;
  ++self.stats_.packets; self.stats_.time_type = p->time_type; self.stats_.last_rx = monotonicSeconds();
  // SDK validates datagram boundaries and any nonzero CRC; selected-device profile may omit CRC.
  if (p->data_type != self.config_.point_data_type) { ++self.stats_.invalid; return; }
  // 统计 CRC 缺失次数；是否放行取决于 DataHandler 的 AllowZeroMid360sPointCrc 设置
  if (p->crc32 == 0) ++self.stats_.crc_missing;
  // 用 droppedPackets 差值归因：本次 addPacket 内部判废的包计入 invalid
  const auto dropped = self.assembler_.droppedPackets();
  self.assembler_.addPacket(*p, 0);
  self.stats_.invalid += self.assembler_.droppedPackets() - dropped;
  ego_sensor::LidarFrame frame;
  if (self.assembler_.popFrame(frame)) {
    // 队列上限固定为小常数：满了丢最旧帧（主循环已落后），并记一次 queue_drops
    if (self.queue_.size() >= self.config_.queue_capacity) { self.queue_.pop_front(); ++self.stats_.queue_drops; }
    self.queue_.push_back(std::move(frame)); ++self.stats_.frames;
  }
}
// 主循环取帧：队列为空返回 false；取走后帧所有权转移给调用方
bool LivoxSource::pop(ego_sensor::LidarFrame &f) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) return false;
  f = std::move(queue_.front()); queue_.pop_front(); return true;
}
// 统计快照：加锁拷贝，调用方拿到的是某一时刻的一致副本
LivoxSource::Stats LivoxSource::stats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
}
