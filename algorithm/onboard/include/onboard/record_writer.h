// 飞行记录/诊断文件：record-dir 下的原始 MAVLink 流与 CSV，以及丢星推算位置文件。
// 所有列顺序只在本模块定义；打开失败一次性返回，运行期写入失败静默忽略（不能影响飞行）。
#ifndef ONBOARD_RECORD_WRITER_H
#define ONBOARD_RECORD_WRITER_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <Eigen/Eigen>
#include <onboard/gps_dead_reckoning.h>
#include <onboard/offboard_pilot.h>

namespace ego_onboard
{
  // timing.csv 一行（字段顺序与 record_writer.cpp 的表头一一对应）。
  struct TimingSample
  {
    double elapsed = 0.0;
    const char *phase = "";
    int gps_fix = 0;
    bool clock_ok = false;
    uint64_t sync_samples = 0;
    double rtt_ms = 0.0, residual_ms = 0.0;
    uint64_t lidar_packets = 0;
    int time_type = 0;
    uint64_t aligned_frames = 0, rejected_frames = 0;
    int map_frames = 0;
    bool pose_fresh = false;
    double max_loop_ms = 0.0;
    uint64_t crc_errors = 0;
    bool ptp_ready = false;
    int64_t lidar_sync_offset_ns = 0;
    uint64_t lidar_crc_missing = 0;
    uint64_t planner_steps = 0;
  };

  class RecordWriter
  {
  public:
    // 打开 record 目录（空串 = 不记录）；建目录/建文件失败返回 false。
    bool open(const std::string &dir, std::string &error);
    // 打开 <dir>/gps_loss.csv（追加）；失败返回 false，由调用方决定是否提示。
    bool openGpsLoss(const std::string &dir, std::string &error);

    // MAVLink 原始流 + 分块索引；偏移由内部维护。
    void rawRx(double now, const uint8_t *data, std::size_t n);
    void rawTx(double now, const uint8_t *data, std::size_t n);
    // 每帧雷达的接纳结果
    void scan(double raw_epoch, double host_monotonic, double receive_monotonic, int time_type,
              std::size_t points, double span_s, bool aligned, bool timing_ready,
              bool crc_verified, bool packet_policy_ok);
    void timing(const TimingSample &s);
    void handover(double elapsed, Phase phase, const HandoverDiagnostics &d);

    // 丢星推算位置：内部 1 Hz 限频，返回 false 表示本拍不采样。
    bool gpsLossDue(double now);
    void gpsLoss(double elapsed, const char *phase, unsigned fix_type, const GeoFix &fix,
                 double age_s, const Eigen::Vector3d &pos);

    // 打开了 record 目录（handover.csv 可用）时返回 true。
    bool recording() const { return handover_.is_open(); }

    // 原始流不每拍 flush（吞吐优先），由 1 Hz 汇报统一落盘。
    void flush();

  private:
    void raw(std::ofstream &bin, std::ofstream &chunks, uint64_t &offset, double now,
             const uint8_t *data, std::size_t n);
    // 打开失败时关掉已建好的文件，避免留下半套空 CSV。
    void closeAll();

    std::ofstream raw_rx_, raw_tx_, rx_chunks_, tx_chunks_;
    std::ofstream timing_, scans_, handover_, gps_loss_;
    uint64_t rx_offset_ = 0, tx_offset_ = 0;
    double last_gps_loss_ = -1e9;
  };

} // namespace ego_onboard

#endif
