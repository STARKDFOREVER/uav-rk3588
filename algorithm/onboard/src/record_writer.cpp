// 记录文件实现：列顺序只在这里出现一次（表头 + 数据行相邻维护）。
#include "onboard/record_writer.h"
#include <cerrno>
#include <iomanip>
#include <sys/stat.h>
#include <onboard/time_alignment.h>

namespace ego_onboard
{
  bool RecordWriter::open(const std::string &dir, std::string &error)
  {
    if (dir.empty()) return true;
    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST)
    {
      error = "cannot create record directory";
      return false;
    }
    raw_rx_.open(dir + "/fcu_rx.bin", std::ios::binary);
    raw_tx_.open(dir + "/fcu_tx.bin", std::ios::binary);
    rx_chunks_.open(dir + "/fcu_rx_chunks.csv");
    tx_chunks_.open(dir + "/fcu_tx_chunks.csv");
    if (!raw_rx_ || !raw_tx_ || !rx_chunks_ || !tx_chunks_)
    {
      error = "cannot open raw MAVLink diagnostic logs";
      closeAll();
      return false;
    }
    timing_.open(dir + "/timing.csv");
    scans_.open(dir + "/scans.csv");
    handover_.open(dir + "/handover.csv");
    if (!timing_ || !scans_ || !handover_)
    {
      error = "cannot open diagnostic logs";
      closeAll();
      return false;
    }
    // 表头：timing.csv 每拍一行；scans.csv 每帧雷达；handover.csv 以 5Hz 采样接管判据。
    timing_ << "elapsed,phase,gps_fix,clock_ok,sync_samples,rtt_ms,residual_ms,lidar_packets,time_type,aligned_frames,rejected_frames,map_frames,pose_fresh,max_loop_ms,crc_errors,ptp_ready,lidar_sync_offset_ns,lidar_crc_missing,planner_steps\n";
    scans_ << "raw_epoch,host_monotonic,receive_monotonic,time_type,points,span_s,aligned,timing_ready,crc_verified,packet_policy_ok\n";
    handover_ << "elapsed,phase,phase_elapsed_s,fc_alive,armed,main_mode,sub_mode,pose_ok,timing_ok,sensing_ok,gps_rx,gps_fix,gps_ok,route_valid,horizontal_error_m,horizontal_position_ok,altitude_rel_m,altitude_ok,speed_mps,speed_ok,ready,ready_stable_s,mode_switch_sent,waiting_offboard,mode_ack_seen,mode_ack_result,mode_ack_age_s\n";
    rx_chunks_ << "host_monotonic,offset,size\n";
    tx_chunks_ << "host_monotonic,offset,size\n";
    return true;
  }

  bool RecordWriter::openGpsLoss(const std::string &dir, std::string &error)
  {
    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST)
    {
      error = "cannot create " + dir;
      return false;
    }
    const std::string path = dir + "/gps_loss.csv";
    struct stat existing{};
    const bool fresh = ::stat(path.c_str(), &existing) != 0 || existing.st_size == 0;
    gps_loss_.open(path, std::ios::app);
    if (!gps_loss_)
    {
      error = "cannot open " + path;
      return false;
    }
    if (fresh) gps_loss_ << "unix_time,elapsed,phase,fix,lat,lon,alt,age,pos_x,pos_y,pos_z\n";
    return true;
  }

  void RecordWriter::closeAll()
  {
    raw_rx_.close();
    raw_tx_.close();
    rx_chunks_.close();
    tx_chunks_.close();
    timing_.close();
    scans_.close();
    handover_.close();
  }

  void RecordWriter::raw(std::ofstream &bin, std::ofstream &chunks, uint64_t &offset, double now,
                         const uint8_t *data, std::size_t n)
  {
    if (!bin) return;
    bin.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(n));
    if (chunks) chunks << std::setprecision(16) << now << ',' << offset << ',' << n << '\n';
    offset += n;
  }

  void RecordWriter::rawRx(double now, const uint8_t *data, std::size_t n)
  {
    raw(raw_rx_, rx_chunks_, rx_offset_, now, data, n);
  }

  void RecordWriter::rawTx(double now, const uint8_t *data, std::size_t n)
  {
    raw(raw_tx_, tx_chunks_, tx_offset_, now, data, n);
  }

  void RecordWriter::scan(double raw_epoch, double host_monotonic, double receive_monotonic,
                          int time_type, std::size_t points, double span_s, bool aligned,
                          bool timing_ready, bool crc_verified, bool packet_policy_ok)
  {
    if (!scans_) return;
    scans_ << std::setprecision(16) << raw_epoch << ',' << host_monotonic << ',' << receive_monotonic
           << ',' << time_type << ',' << points << ',' << span_s << ',' << aligned << ','
           << timing_ready << ',' << crc_verified << ',' << packet_policy_ok << '\n';
  }

  void RecordWriter::timing(const TimingSample &s)
  {
    if (!timing_) return;
    timing_ << std::setprecision(12) << s.elapsed << ',' << s.phase << ',' << s.gps_fix << ','
            << s.clock_ok << ',' << s.sync_samples << ',' << s.rtt_ms << ',' << s.residual_ms << ','
            << s.lidar_packets << ',' << s.time_type << ',' << s.aligned_frames << ','
            << s.rejected_frames << ',' << s.map_frames << ',' << s.pose_fresh << ','
            << s.max_loop_ms << ',' << s.crc_errors << ',' << s.ptp_ready << ','
            << s.lidar_sync_offset_ns << ',' << s.lidar_crc_missing << ',' << s.planner_steps << '\n';
    timing_.flush();
  }

  void RecordWriter::handover(double elapsed, Phase phase, const HandoverDiagnostics &d)
  {
    if (!handover_) return;
    handover_ << std::setprecision(12) << elapsed << ',' << phaseName(phase) << ',' << d.phase_elapsed_s << ','
              << d.fc_alive << ',' << d.armed << ',' << unsigned(d.main_mode) << ',' << unsigned(d.sub_mode) << ','
              << d.pose_ok << ',' << d.timing_ok << ',' << d.sensing_ok << ',' << d.gps_rx << ','
              << unsigned(d.gps_fix_type) << ',' << d.gps_ok << ',' << d.route_valid << ','
              << d.horizontal_error_m << ',' << d.horizontal_position_ok << ',' << d.altitude_rel_m << ','
              << d.altitude_ok << ',' << d.speed_mps << ',' << d.speed_ok << ',' << d.ready << ','
              << d.ready_stable_s << ',' << d.mode_switch_sent << ',' << d.waiting_offboard << ','
              << d.mode_ack_seen << ',' << unsigned(d.mode_ack_result) << ',' << d.mode_ack_age_s << '\n';
    handover_.flush();
  }

  bool RecordWriter::gpsLossDue(double now)
  {
    if (!gps_loss_ || now - last_gps_loss_ < 1.0) return false;
    last_gps_loss_ = now;
    return true;
  }

  void RecordWriter::gpsLoss(double elapsed, const char *phase, unsigned fix_type,
                             const GeoFix &fix, double age_s, const Eigen::Vector3d &pos)
  {
    if (!gps_loss_) return;
    gps_loss_ << std::setprecision(12) << unixSeconds() << ',' << elapsed << ',' << phase << ','
              << fix_type << ',' << fix.lat_deg << ',' << fix.lon_deg << ',' << fix.alt_m << ','
              << age_s << ',' << pos.x() << ',' << pos.y() << ',' << pos.z() << '\n';
    gps_loss_.flush();
  }

  void RecordWriter::flush()
  {
    if (raw_rx_) raw_rx_.flush();
    if (raw_tx_) raw_tx_.flush();
    if (rx_chunks_) rx_chunks_.flush();
    if (tx_chunks_) tx_chunks_.flush();
  }

} // namespace ego_onboard
