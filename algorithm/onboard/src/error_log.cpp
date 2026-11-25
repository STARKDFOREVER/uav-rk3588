// 错误日志实现：单文件追加 + 进程内互斥，保证主线程与规划/雷达回调线程并发写不交错。
#include "onboard/error_log.h"
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <sys/stat.h>

namespace ego_onboard
{
  namespace
  {
    std::mutex &logMutex()
    {
      static std::mutex mutex;
      return mutex;
    }
    std::string &logDir()
    {
      static std::string dir;
      return dir;
    }
    // 逐级创建目录；已存在（EEXIST）视为成功。
    bool ensureDir(const std::string &dir)
    {
      for (std::size_t i = 1; i <= dir.size(); ++i)
      {
        if (i != dir.size() && dir[i] != '/') continue;
        const std::string part = dir.substr(0, i);
        if (::mkdir(part.c_str(), 0755) != 0 && errno != EEXIST) return false;
      }
      return true;
    }
  }

  void setErrorLogDir(const std::string &dir)
  {
    std::lock_guard<std::mutex> lock(logMutex());
    logDir() = dir;
  }

  void logError(const std::string &message)
  {
    std::lock_guard<std::mutex> lock(logMutex());
    if (logDir().empty() || !ensureDir(logDir())) return;
    std::FILE *file = std::fopen((logDir() + "/onboard_error.log").c_str(), "a");
    if (!file) return;
    const std::time_t now = std::time(nullptr);
    char stamp[32] = {};
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    std::fprintf(file, "[%s] %s\n", stamp, message.c_str());
    std::fclose(file);
  }

  void redirectStderrToErrorLog()
  {
    std::lock_guard<std::mutex> lock(logMutex());
    if (logDir().empty() || !ensureDir(logDir())) return;
    std::freopen((logDir() + "/onboard_error.log").c_str(), "a", stderr);
  }
}
