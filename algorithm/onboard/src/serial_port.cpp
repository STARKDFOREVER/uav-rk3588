#include "serial_port.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <chrono>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <termios.h>
#include <unistd.h>

namespace ego_onboard
{

  // 波特率白名单：只映射 termios 能表达的标准速率，未命中返回 0，
  // 由 open() 报 unsupported baud rate（避免 B0 被误解为挂断线路）
  static speed_t baudToSpeed(int baud)
  {
    switch (baud)
    {
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
    }
  }

  // 析构即关闭：确保读线程被 join，不会访问已析构的 this
  SerialPort::~SerialPort() { close(); }

  bool SerialPort::open(const std::string &dev, int baud, std::string &err)
  {
    // 重复 open 等价于重连：先停旧读线程、释放旧描述符
    close();
    // 前缀 tcp:// 表示串口服务器（TCP 桥接），否则按本地串口设备打开
    const bool tcp = dev.compare(0, 6, "tcp://") == 0;
    if (tcp) {
      // host 与 port 以最后一个冒号分隔（IPv4 地址与主机名本身不含冒号）
      const auto colon = dev.find_last_of(':');
      if (colon <= 6 || colon == std::string::npos) { err="expected tcp://host:port"; return false; }
      const std::string host=dev.substr(6, colon-6), port=dev.substr(colon+1);
      // 只解析一次地址；失败说明主机名/IP 写错，不重试
      addrinfo hints{}, *res=nullptr; hints.ai_socktype=SOCK_STREAM;
      if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res)) { err="TCP bridge address lookup failed"; return false; }
      // 依次尝试解析出的候选地址；SOCK_NONBLOCK 保证 connect 不阻塞调用线程
      for (auto *a=res; a; a=a->ai_next) {
        fd_=::socket(a->ai_family, a->ai_socktype | SOCK_NONBLOCK, a->ai_protocol);
        if (fd_<0) continue;
        // connect 返回 EINPROGRESS 时最多等 1 s 完成握手，再用 SO_ERROR 确认是否真连上
        int rc=::connect(fd_,a->ai_addr,a->ai_addrlen);
        pollfd pfd{fd_, POLLOUT, 0}; int e=0; socklen_t n=sizeof(e);
        if (rc==0 || (errno==EINPROGRESS && ::poll(&pfd,1,1000)>0 && getsockopt(fd_,SOL_SOCKET,SO_ERROR,&e,&n)==0 && e==0)) break;
        // 该候选地址连不上：关掉后尝试下一个
        ::close(fd_); fd_=-1;
      }
      freeaddrinfo(res);
      // 关闭 Nagle：MAVLink 全是小包，攒包会额外引入数十毫秒时延
      if(fd_>=0) { int yes=1; setsockopt(fd_,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes)); }
    } else fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    // 串口与 TCP 桥接共用错误出口：失败时 err 带上 strerror 便于现场排查
    if (fd_ < 0)
    {
      err = std::string("open(") + dev + ") 失败: " + std::strerror(errno);
      return false;
    }

    if (!tcp) {
    // 串口配置：下列任一 tcgetattr/cfsetispeed/cfsetospeed/tcsetattr 失败都要 close() 回滚后报错
    termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0)
    {
      err = std::string("tcgetattr 失败: ") + std::strerror(errno);
      close();
      return false;
    }
    cfmakeraw(&tio); // 8N1、无回显、无行缓冲
    // 波特率必须在白名单内，否则 termios 无法表达，直接报错
    const speed_t spd = baudToSpeed(baud);
    if (!spd) { err="unsupported baud rate"; close(); return false; }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS; // 不用硬件流控
    // VMIN/VTIME 置 0：read 不做字符数与超时等待，实际读节奏由读线程的 poll 控制
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
      err = std::string("tcsetattr 失败: ") + std::strerror(errno);
      close();
      return false;
    }
    // 丢弃打开瞬间残留的半帧，避免 MAVLink 解析器从错位字节开始同步
    tcflush(fd_, TCIOFLUSH);
    }

    // 启动读线程：它是唯一读该描述符的地方，写入统一走 write()；
    // 置 run_ 必须早于建线程，否则读线程可能刚起来就看到 false 而立即退出
    run_ = true;
    reader_ = std::thread([this]() {
      uint8_t buf[1024];
      while (run_)
      {
        // Wait for readiness rather than adding a fixed 2 ms to each response.
        pollfd ready{fd_, POLLIN, 0};
        // 10 ms 超时只为让循环能及时看到 run_ 变 false（决定退出延迟），不是采样间隔
        const int rc = ::poll(&ready, 1, 10);
        // EINTR 是信号打断，继续等；其余负值、挂断、描述符失效都视为链路断开，退出线程
        if (rc < 0 && errno == EINTR) continue;
        if (rc < 0) break;
        if (rc == 0) continue;
        if (!(ready.revents & POLLIN) && (ready.revents & (POLLERR | POLLHUP | POLLNVAL))) break;
        // 一次最多读 1 KiB，字节原样入缓冲，分帧由上层 MAVLink 解析器完成
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0)
        {
          // 加锁追加；超 1 MiB 说明消费端停摆，丢弃最旧数据以保证内存有界
          std::lock_guard<std::mutex> lk(mtx_);
          rx_.insert(rx_.end(), buf, buf + n);
          if (rx_.size() > (1u << 20)) // 没人取走时别无限涨
            rx_.erase(rx_.begin(), rx_.begin() + (rx_.size() - (1u << 20)));
        }
        // n==0 表示对端关闭（TCP）或设备消失；EAGAIN/EWOULDBLOCK/EINTR 是正常无数据；
        // 其余错误不可恢复，跳出循环
        else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
          break;
      }
      // 读线程自行退出时也要清标志，使 isOpen() 立即反映链路已断，供上层触发重连
      run_ = false;
    });
    return true;
  }

  void SerialPort::close()
  {
    // 先置标志再 join：读线程最多等一个 poll 超时即退出；
    // 必须 join 完成后才 close(fd_)，否则描述符可能被别的线程复用
    run_ = false;
    if (reader_.joinable())
      reader_.join();
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
    // 读线程已退出、fd 不再被使用，此时关闭才安全；最后清空残留字节，重连不串旧帧
    std::lock_guard<std::mutex> lk(mtx_);
    rx_.clear();
  }

  void SerialPort::take(std::vector<uint8_t> &out)
  {
    // 用 swap 转移所有权：临界区内只交换一次指针，耗时与数据量无关
    std::lock_guard<std::mutex> lk(mtx_);
    out.swap(rx_);
    rx_.clear();
  }

  bool SerialPort::write(const uint8_t *data, size_t len)
  {
    // 未打开、或读线程已退出（链路已断）都不发，由上层决定是否重连
    if (fd_ < 0 || !run_)
      return false;
    // 整帧 20 ms 总预算：耗尽即返回 false（可能已写出部分字节，不保证原子性）
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(20);
    size_t off = 0;
    while (off < len)
    {
      // 非阻塞写：EINTR 重试，写成功推进偏移，EAGAIN 才转去等可写
      const ssize_t n = ::write(fd_, data + off, len - off);
      if (n < 0 && errno == EINTR) continue;
      if (n > 0)
      {
        off += (size_t)n;
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        pollfd pfd{fd_, POLLOUT, 0};
        if (::poll(&pfd,1,2)<0 && errno!=EINTR) return false;
        continue;
      }
      return false;
    }
    // 全部字节已交给内核发送缓冲（不等于对端已收到）
    return true;
  }

} // namespace ego_onboard
