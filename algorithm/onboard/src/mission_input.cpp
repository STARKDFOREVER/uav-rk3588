// 本地 KML 任务输入：等待文件出现 -> 读取 -> 解析第一条 LineString -> 固定高度航线。
// 解析使用 libxml2 的内存解析，禁用网络与实体展开（XML_PARSE_NONET），
// 拒绝含 DTD 的文档，避免 XXE 与外部实体攻击。
// 坐标约定：KML 的 tuple 为 "lon,lat,alt"（经度在前），只取经纬度，
// 内部统一转成 GeoWaypoint 的 (lat, lon, constant_alt_m)，高度由配置固定给出。
// 所有数量上限（max_waypoints / max_bytes）都按上限值传入各层，避免大文件耗尽内存。
#include "onboard/mission_input.h"
#include "onboard/route_utils.h"
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <cerrno>
#include <climits>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <sys/stat.h>

namespace ego_onboard
{
  namespace
  {
    // 单个坐标点：只保留经纬度；坐标第 3 分量（高度）不再参与计算。
    struct Sample
    {
      double lon = 0, lat = 0;
    };

    // 匹配元素名，且命名空间为空或官方 KML 2.2 命名空间，避免误匹配其它扩展元素。
    bool named(xmlNode *node, const char *name)
    {
      if (!node || node->type != XML_ELEMENT_NODE || !xmlStrEqual(node->name, BAD_CAST name))
        return false;
      return !node->ns || !node->ns->href ||
             xmlStrEqual(node->ns->href, BAD_CAST "http://www.opengis.net/kml/2.2");
    }
    // 取节点文本内容（含子节点），调用方负责语义解析
    std::string content(xmlNode *node)
    {
      if (!node) return {};
      xmlChar *value = xmlNodeGetContent(node);
      std::string result = value ? reinterpret_cast<const char *>(value) : "";
      xmlFree(value);
      return result;
    }
    // 找第一个同名子元素（只看直接子节点，不做递归）
    xmlNode *child(xmlNode *node, const char *name)
    {
      for (xmlNode *c = node ? node->children : nullptr; c; c = c->next)
        if (named(c, name)) return c;
      return nullptr;
    }
    // 去掉首尾空白（KML 常带换行/制表符缩进）
    std::string trim(const std::string &s)
    {
      const auto a = s.find_first_not_of(" \t\r\n");
      if (a == std::string::npos) return {};
      return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
    }
    // 严格数值解析：必须整串都是数字且不溢出。
    bool number(const std::string &text, double &value)
    {
      const std::string s = trim(text);
      if (s.empty()) return false;
      errno = 0;
      char *end = nullptr;
      value = std::strtod(s.c_str(), &end);
      return end == s.c_str() + s.size() && errno != ERANGE && std::isfinite(value);
    }
    // 解析 <coordinates>：空白分隔多个 tuple，每个 tuple 为 "lon,lat[,alt]"。
    // 第 3 分量允许存在但被忽略；limit 同时充当结果数量上限，超限立即报错。
    bool coordinates(xmlNode *geometry, std::size_t limit, std::vector<Sample> &result,
                     std::string &error)
    {
      xmlNode *coordinate_node = child(geometry, "coordinates");
      if (!coordinate_node) { error = "KML geometry has no coordinates"; return false; }
      std::istringstream input(content(coordinate_node));
      std::string token;
      std::size_t tuples = 0;
      while (input >> token)
      {
        ++tuples;
        // 按逗号切分 tuple：2 或 3 个字段（经度、纬度、可选高度）
        std::vector<std::string> fields;
        std::size_t offset = 0;
        for (;;)
        {
          auto comma = token.find(',', offset);
          fields.push_back(token.substr(offset, comma == std::string::npos ? comma : comma - offset));
          if (comma == std::string::npos) break;
          offset = comma + 1;
        }
        Sample sample;
        if (fields.size() < 2 || fields.size() > 3 ||
            !number(fields[0], sample.lon) || !number(fields[1], sample.lat))
        { error = "Invalid KML coordinate tuple"; return false; }
        if (std::abs(sample.lon) > 180 || std::abs(sample.lat) > 90)
        { error = "KML longitude/latitude is outside its valid range"; return false; }
        if (result.size() >= limit) { error = "KML has too many coordinates"; return false; }
        result.push_back(sample);
      }
      if (tuples == 0) { error = "KML coordinates are empty"; return false; }
      return true;
    }
    // 递归找文档中第一条 LineString 的坐标；Point / MultiGeometry 一律忽略。
    bool firstLineString(xmlNode *node, std::size_t limit, std::vector<Sample> &points,
                         bool &found, std::string &error)
    {
      for (; node; node = node->next)
      {
        if (named(node, "LineString"))
        {
          if (!coordinates(node, limit, points, error)) return false;
          found = true;
          return true;
        }
        if (!firstLineString(node->children, limit, points, found, error)) return false;
        if (found) return true;
      }
      return true;
    }
    // 配置合法性前置校验：上限必须是正数且不超过 INT_MAX（libxml2 用 int 表示长度）；
    // max_waypoints 至少 2（起飞点 + 至少一个导航点），上限 65533 与飞控航点协议容量对齐。
    bool configValid(const MissionInputConfig &c, std::string &error)
    {
      if (c.max_bytes == 0 || c.max_bytes > static_cast<std::size_t>(INT_MAX) ||
          c.max_waypoints < 2 || c.max_waypoints > 65533 ||
          !std::isfinite(c.min_alt_m) || !std::isfinite(c.max_alt_m) ||
          c.min_alt_m < 0 || c.max_alt_m <= c.min_alt_m)
      { error = "Invalid mission_input limits"; return false; }
      // 固定高度必须落在 (min_alt_m, max_alt_m] 内。
      if (!std::isfinite(c.constant_alt_m) ||
          c.constant_alt_m <= c.min_alt_m || c.constant_alt_m > c.max_alt_m)
      { error = "constant_alt_m outside mission_input limits"; return false; }
      return true;
    }

    // 分块读取整个文件；每块都按剩余额度检查，超限即中止，避免一次性读入超大文件。
    bool loadFile(const MissionInputConfig &c, std::string &body, std::string &error)
    {
      std::ifstream file(c.file, std::ios::binary);
      if (!file) { error = "Cannot open local KML: " + c.file; return false; }
      char chunk[8192];
      body.clear();
      while (file.read(chunk, sizeof(chunk)) || file.gcount())
      {
        const auto n = static_cast<std::size_t>(file.gcount());
        if (n > c.max_bytes - body.size()) { error = "KML exceeds max_bytes"; return false; }
        body.append(chunk, n);
      }
      if (!file.eof()) { error = "Local KML read failed"; return false; }
      return true;
    }

    // 等待模式下只判断文件是否已就位，不做版本签名或稳定性检查。
    bool fileReady(const std::string &path, std::string &error)
    {
      struct stat info;
      if (::stat(path.c_str(), &info) != 0)
      { error = "Waiting for local KML: " + path; return false; }
      if (!S_ISREG(info.st_mode))
      { error = "KML must be a regular file: " + path; return false; }
      return true;
    }

  }

  const char *missionInputStateName(MissionInputState state)
  {
    switch (state)
    {
    case MissionInputState::IDLE: return "IDLE";
    case MissionInputState::WAIT_KML: return "WAIT_KML";
    case MissionInputState::LOADING_KML: return "LOADING_KML";
    case MissionInputState::PARSING_KML: return "PARSING_KML";
    case MissionInputState::READY: return "READY";
    case MissionInputState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
  }

  bool parseMissionKml(const std::string &xml, const MissionInputConfig &config,
                       PreparedMission &mission, std::string &error)
  {
    mission = PreparedMission{};
    error.clear();
    if (!configValid(config, error)) return false;
    if (xml.empty() || xml.size() > config.max_bytes)
    { error = "KML is empty or exceeds max_bytes"; return false; }
    // NONET 禁止网络加载，NOBLANKS 去掉纯空白节点；后续再显式拒绝 DTD/外部子集。
    std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
        xmlReadMemory(xml.data(), static_cast<int>(xml.size()), "mission.kml", nullptr,
                      XML_PARSE_NONET | XML_PARSE_NOBLANKS | XML_PARSE_NOERROR | XML_PARSE_NOWARNING), xmlFreeDoc);
    if (!doc || doc->intSubset || doc->extSubset || !named(xmlDocGetRootElement(doc.get()), "kml"))
    { error = "Expected valid KML XML without DTD / external entities"; return false; }
    std::vector<Sample> points;
    bool found = false;
    if (!firstLineString(xmlDocGetRootElement(doc.get()), config.max_waypoints, points, found, error))
      return false;
    if (!found) { error = "KML has no LineString route"; return false; }
    if (points.empty() || points.size() > config.max_waypoints)
    { error = "KML has no route or too many waypoints"; return false; }
    // 高度全程固定为配置值（configValid 已保证落在允许范围内）。
    for (const auto &sample : points)
      mission.route.push_back({sample.lat, sample.lon, config.constant_alt_m});
    // 自动追加返航段；返航点同样受 max_waypoints 上限约束。
    bool return_leg_added = false;
    return ensureReturnToStart(mission.route, config.max_waypoints, return_leg_added, error);
  }

  // 启动入口：清空上次状态并按 wait_for_file 决定初始状态。
  // 配置非法或 file 为空时直接进 FAILED，便于启动阶段快速暴露配置错误。
  void MissionInput::start(const MissionInputConfig &config)
  {
    config_ = config;
    mission_ = PreparedMission{};
    body_.clear(); error_.clear();
    state_ = config.wait_for_file ? MissionInputState::WAIT_KML : MissionInputState::LOADING_KML;
    if (!configValid(config_, error_)) state_ = MissionInputState::FAILED;
    else if (config_.file.empty())
    {
      error_ = "mission_input.file is required";
      state_ = MissionInputState::FAILED;
    }
  }

  void MissionInput::step()
  {
    step(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  // 失败终态：清空半成品任务与文件内容，不再重试。
  void MissionInput::rejectOrFail()
  {
    mission_ = PreparedMission{};
    body_.clear();
    state_ = MissionInputState::FAILED;
  }

  // 每次 step 只推进一步状态，全部工作都在主循环线程完成，避免在回调里做重活。
  void MissionInput::step(double /*now*/)
  {
    switch (state_)
    {
    case MissionInputState::WAIT_KML:
    {
      // 只等文件出现：存在即读取，不做版本签名或写入稳定判断。
      std::string error;
      if (!fileReady(config_.file, error))
      { error_ = error; break; }
      error_.clear();
      state_ = MissionInputState::LOADING_KML;
      break;
    }
    case MissionInputState::LOADING_KML:
      if (!loadFile(config_, body_, error_)) rejectOrFail();
      else state_ = MissionInputState::PARSING_KML;
      break;
    case MissionInputState::PARSING_KML:
      // 解析成功即进入 READY 并锁定任务；失败直接终态。
      if (!parseMissionKml(body_, config_, mission_, error_)) rejectOrFail();
      else { state_ = MissionInputState::READY; error_.clear(); }
      // 解析完即释放正文，避免长期占用内存（最大 max_bytes）
      body_.clear();
      break;
    default: break;
    }
  }
}
