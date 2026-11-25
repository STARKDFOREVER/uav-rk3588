// ---------------------------------------------------------------------------
// mission_inbox_selftest.cpp —— KML 任务收件箱（MissionInput）自测，CTest 单元级
// 测试目的：在真实文件系统上驱动 MissionInput 状态机，验证“先到先装、装后锁存”：
//   缺失 KML 时持续等待、出现后立即读取、非法内容直接 FAILED、合法任务自动闭合、
//   READY 后替换内容无法改写已激活航线、restart 清空旧任务，以及严格模式快速失败。
// 覆盖约束：READY 锁存；失败为终态；非法任务上限（max_waypoints=0）必须显式拒绝；
//   wait_for_file=false 时缺文件立即 FAILED。
// 运行参数：无（进程内构造配置）；临时目录由 mkdtemp 创建，测试结束删除。
// 通过判据：每条 check 打印 [PASS]/[FAIL]，存在失败时退出码 1；mkdtemp 失败退出 2。
// 说明：step() 的时间参数是合成的单调秒，用于在毫秒内模拟等待/解析时序，
//   不需要真实等待；文件内容通过覆盖写入模拟直传投递。
// ---------------------------------------------------------------------------
#include <onboard/mission_input.h>
#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace ego_onboard;
namespace {
// 合法 KML 夹具：LineString 含 2 个坐标，解析器会补闭合点，故下方断言 route.size()==3。
const char *valid = "<kml><Placemark><LineString><coordinates>113,21,0 113.001,21,0</coordinates></LineString></Placemark></kml>";
// 覆盖式写入工具：既用于半成品/非法替换，也用于写 .tmp 暂存文件。
void write(const std::string &path, const std::string &text) { std::ofstream out(path); out << text; }
}
// failures 汇总所有断言，最终决定进程退出码。
int main() {
  int failures=0;
  // 统一断言输出：打印 [PASS]/[FAIL]，CTest 日志可直接定位失败项。
  auto check=[&](bool ok,const char *name){std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);if(!ok)++failures;};
  // mkdtemp 生成唯一目录，避免并行用例共用收件箱；失败属于环境异常，故返回 2 而非断言失败。
  char directory[]="/tmp/uav-mission-inbox-XXXXXX";
  if(!mkdtemp(directory)) return 2;
  // path 是正式收件箱；temporary 是同目录 .tmp，用于模拟生产端原子 rename 投递。
  const std::string path=std::string(directory)+"/mission.kml", temporary=path+".tmp";
  // wait_for_file=true：文件缺失属于正常等待，不应立即失败。
  MissionInputConfig cfg;cfg.file=path;cfg.wait_for_file=true;
  MissionInput input;input.start(cfg);
  // 合成时间 1s→1000s：长时间无文件仍应保持 WAIT_KML 且不产生任务。
  input.step(1);input.step(1000);
  check(input.state()==MissionInputState::WAIT_KML&&input.mission().route.empty(),"missing KML waits indefinitely without a task");
  // 写入“有 XML 但结构不完整”的内容，模拟传输中的半成品。
  write(path,"<kml>");input.step(1001);input.step(1001.4);
  check(input.state()==MissionInputState::PARSING_KML,"a file is read immediately when it appears");
  // 解析失败是终态：错误被记录，任务保持为空。
  input.step(1001.6);
  check(input.state()==MissionInputState::FAILED&&!input.error().empty()&&input.mission().route.empty(),"invalid KML fails terminally without a task");
  // 重新 start 后再投递合法文件：应自动装载一条闭合任务。
  input.start(cfg);
  write(temporary,valid);std::rename(temporary.c_str(),path.c_str());
  input.step(1011);input.step(1011.6);input.step(1011.6);
  check(input.state()==MissionInputState::READY&&input.mission().route.size()==3,"valid delivery starts one closed task automatically");
  // 保存已接受航点，供后面验证 READY 锁存不会被新文件内容改写。
  const auto saved=input.mission().route;
  // READY 后直接覆盖收件箱为非法内容：已激活任务必须保持不变（锁存语义）。
  write(path,"broken replacement");input.step(1012);
  check(input.state()==MissionInputState::READY&&input.mission().route.size()==saved.size()&&input.mission().route[1].lon_deg==saved[1].lon_deg,"READY latches the task; replacement cannot change the active route");
  // 重新装载同一份合法文件：解析与固定高度装配一步完成，直接进入 READY。
  write(path,valid);input.start(cfg);input.step(2000);input.step(2001);input.step(2001);
  check(input.state()==MissionInputState::READY&&input.mission().route.size()==3,"restart reloads the valid task in one parse pass");
  // 删除文件后 restart：start 必须清空上一次准备的任务，而不是复用旧状态。
  std::remove(path.c_str());input.start(cfg);input.step(3000);
  check(input.state()==MissionInputState::WAIT_KML&&input.mission().route.empty(),"restart clears any previous prepared task");
  // 关闭等待：prepare-only 严格模式下缺文件应立即 FAILED，便于上层快速探错。
  cfg.wait_for_file=false;input.start(cfg);input.step();
  check(input.state()==MissionInputState::FAILED,"strict prepare-only mode still fails immediately on a missing file");
  // 非法任务上限（0 个航点）必须显式拒绝，不能被静默接受成空任务。
  cfg.wait_for_file=true;cfg.max_waypoints=0;input.start(cfg);
  check(input.state()==MissionInputState::FAILED,"invalid mission limits are not silently accepted");
  // 清理临时目录；有断言失败则返回 1，否则 0 表示 CTest 通过。
  rmdir(directory);
  return failures?1:0;
}
