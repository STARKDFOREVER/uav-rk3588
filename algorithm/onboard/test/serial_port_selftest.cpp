// ---------------------------------------------------------------------------
// serial_port_selftest.cpp —— SerialPort 在 PTY 上的收发自测（CTest / 台架工具）
// 测试目的：用 openpty 伪造串口，验证 SerialPort::open/take/write/isOpen 在 921600
//   波特下的收发完整性，以及对端关闭后能检测断开并拒绝写入。
// 覆盖约束：8192 字节模式必须逐字节一致（rx）；write 的 128 字节必须原样到达 PTY
//   主端（tx）；主端关闭后 isOpen()==false 且 write() 返回 false（disconnect）。
// 运行参数：无（PTY 由内核分配，无需真实硬件）。
// 通过判据：rx&&tx&&disconnect 时退出码 0；openpty 或 open 失败返回 1。
// 说明：主端设为 O_NONBLOCK，循环内 sleep 1ms 让从端有调度机会；1s steady_clock
//   截止时间仅为防止收发互等挂死，不是吞吐/实时性指标。
// ---------------------------------------------------------------------------
#include "serial_port.h"
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <thread>
#include <cstdio>
using namespace ego_onboard;
// 用 openpty 伪造一对主/从端：从端路径 name 交给 SerialPort，主端模拟对端设备。
int main() {
  int master=-1,slave=-1;char name[128];
  if(openpty(&master,&slave,name,nullptr,nullptr)) return 1;
  // 关闭本进程持有的从端句柄，让 SerialPort 独占；主端非阻塞以免测试读写互相等待。
  close(slave);fcntl(master,F_SETFL,O_NONBLOCK);
  // 生产代码使用的 SerialPort 实例；错误信息写入 error 供失败时打印。
  SerialPort port;std::string error;
  // 921600 为机载链路常用波特率；打开失败属环境问题，直接返回 1。
  if(!port.open(name,921600,error)) {std::puts(error.c_str());close(master);return 1;}
  // 8192 字节非重复模式（i*37）便于发现错位/截断，全零会掩盖问题。
  std::vector<uint8_t> sent(8192),received,chunk;
  for(size_t i=0;i<sent.size();++i)sent[i]=uint8_t(i*37);
  // 1s 截止时间只用于防止收发互相等待导致挂死，不是吞吐量断言。
  size_t off=0;const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  // 双向推进：主端分片写入、SerialPort 侧 take 取走并累积，直到全部收到或超时。
  while((off<sent.size()||received.size()<sent.size())&&std::chrono::steady_clock::now()<deadline) {
    if(off<sent.size()){ssize_t n=::write(master,sent.data()+off,sent.size()-off);if(n>0)off+=size_t(n);}
    port.take(chunk);received.insert(received.end(),chunk.begin(),chunk.end());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // 严格逐字节比较：任何丢失、重复或顺序错误都判失败。
  bool rx=sent==received;
  const bool wrote=port.write(sent.data(),128);std::vector<uint8_t> reply;
  // 最多 200×1ms 轮询主端，等待 SerialPort::write 的 128 字节出现在 PTY 上。
  for(int i=0;i<200&&reply.size()<128;++i) {
    uint8_t data[128];const ssize_t n=read(master,data,sizeof(data));if(n>0)reply.insert(reply.end(),data,data+n);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // 长度与内容必须同时匹配，否则视为发送路径失败。
  bool tx=wrote&&reply.size()==128&&std::equal(reply.begin(),reply.end(),sent.begin());
  // 关闭主端模拟对端断开；短睡 50ms 给 SerialPort 感知 POLLHUP/EIO。
  close(master);std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // 断开后必须同时 isOpen()==false 且 write() 返回 false，避免误报链路可用。
  bool disconnect=!port.isOpen()&&!port.write(sent.data(),1);port.close();
  // 打印实测收到的字节数，便于定位 PTY 缓冲或丢字节问题。
  std::printf("PTY receive bytes=%zu exact=%d transmit=%d disconnected write rejected=%d\n",received.size(),rx,tx,disconnect);
  // 三项全真才通过。
  return rx&&tx&&disconnect?0:1;
}
