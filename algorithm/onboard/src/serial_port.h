#ifndef _ONBOARD_SERIAL_PORT_H_
#define _ONBOARD_SERIAL_PORT_H_

// POSIX 串口收发：读线程把收到的字节攒进缓冲，主循环定期取走。
// 用途：机载计算机用 USB 串口直连飞控（硬件在环 / 实飞）。
// 承载的字节流通常是 MAVLink v2 帧；本类只做字节搬运，不做分帧与校验。
// 线程模型：open() 成功后启动一个后台读线程，它是唯一读设备的地方；take()/write()/
// isOpen() 建议由同一个主循环线程调用。rx_ 由 mtx_ 保护，fd_ 与 run_ 不跨线程竞争。
// 设备名两种形态：/dev/ttyUSB0 之类的真实串口，或 tcp://host:port 的串口服务器。
// 重连语义：链路异常时读线程自行退出并把 run_ 置 false（isOpen() 随之为 false），
// 由上层销毁重建或再次 open()；本类内部不做自动重试与退避。
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ego_onboard
{

  class SerialPort
  {
  public:
    // 析构自动 close()，保证读线程被 join 后再释放成员
    ~SerialPort();
    // 打开设备并按 baud 配置 8N1；失败时 err 填具体原因（含 strerror），返回 false。
    // 打开全程非阻塞：串口带 O_NONBLOCK，TCP 桥接用非阻塞 connect + poll 等握手，
    // 因此即使对端不在线也不会把调用线程卡住。重复调用等价于重连。
    bool open(const std::string &dev, int baud, std::string &err);
    // 停读线程 -> join -> 关 fd -> 清空缓冲，可重复调用（幂等）
    void close();
    // 只有 fd 有效且读线程仍在跑才算打开；读线程因错误退出后此值为 false，可用于触发重连
    bool isOpen() const { return fd_ >= 0 && run_; }
    // 取走并清空接收缓冲；没有新数据时 out 为空。调用方负责把字节流交给 MAVLink 解析
    void take(std::vector<uint8_t> &out);
    // 发送整帧字节：成功返回 true。带 20 ms 总超时，超时/部分写出/链路断开返回 false，
    // 返回 false 时可能已写出部分字节，上层需整帧重发。
    bool write(const uint8_t *data, size_t len);

  private:
    // 设备或套接字描述符，-1 表示未打开
    int fd_ = -1;
    // 读线程运行标志：close() 置 false，读线程遇到致命错误也会自行清掉
    std::atomic<bool> run_{false};
    // 后台接收线程，生命周期由 open()/close() 管理
    std::thread reader_;
    // 保护 rx_：读线程追加，take() 取走
    std::mutex mtx_;
    // 接收字节缓冲，上限 1 MiB，消费端停摆时丢弃最旧数据保证内存有界
    std::vector<uint8_t> rx_;
  };

} // namespace ego_onboard

#endif
