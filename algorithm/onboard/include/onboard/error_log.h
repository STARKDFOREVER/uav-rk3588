// 运行期错误日志：所有错误统一追加到 <error_dir>/onboard_error.log，不向控制台输出。
// 目录在启动时设置一次；写文件失败时静默忽略（错误处理本身不应影响飞行主流程）。
#ifndef ONBOARD_ERROR_LOG_H
#define ONBOARD_ERROR_LOG_H

#include <string>

namespace ego_onboard
{
  // 设置错误日志目录；传空串表示禁用文件日志。
  void setErrorLogDir(const std::string &dir);
  // 追加一条带本地时间戳的错误记录。
  void logError(const std::string &message);
  // 把 stderr 重定向到错误日志，一并承接 EGO/SDK 等第三方库的错误输出。
  // 需在启动线程之前调用；目录不可创建时保持原 stderr 不变。
  void redirectStderrToErrorLog();
}
#endif
