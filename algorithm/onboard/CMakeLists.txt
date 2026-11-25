# 机载 onboard 主程序构建脚本：依赖同仓库的规划核心、传感器解析与 Livox SDK2。
cmake_minimum_required(VERSION 3.10)
project(ego_onboard CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 未指定构建类型时默认 Release，保证机上运行性能。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 三个必需依赖：Eigen 线性代数、yaml-cpp 读取 onboard.yaml、LibXml2 解析 KML 任务。
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(LibXml2 REQUIRED)
# 依赖源码目录：规划核心、传感器编解码、MAVLink 头文件，以及仓库内 lidar/Livox-SDK2。
set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ego_planner_core)
set(SENSOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sensor)
set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../mavlink)
set(LIVOX_SDK2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lidar/Livox-SDK2)
# 配置阶段即校验 SDK 头文件与 CRC 补丁：缺失时运行期才会静默丢点，无法事后补救，
# 因此直接 FATAL_ERROR 终止构建，并提示执行 lidar/setup_sdk.sh。
if(NOT EXISTS "${LIVOX_SDK2_DIR}/sdk_core/data_handler/data_handler.h")
  message(FATAL_ERROR "Livox SDK missing. Run: bash lidar/setup_sdk.sh (from project root)")
endif()
file(READ "${LIVOX_SDK2_DIR}/sdk_core/data_handler/data_handler.h" LIVOX_DATA_HANDLER_HEADER)
if(NOT LIVOX_DATA_HANDLER_HEADER MATCHES "AllowZeroMid360sPointCrc")
  message(FATAL_ERROR "Livox SDK project patch missing. Run: bash lidar/setup_sdk.sh")
endif()
# 全局头文件搜索路径：本模块、规划核心、传感器、MAVLink 与 SDK2 头文件。
include_directories(${EIGEN3_INCLUDE_DIR} include ${CORE_DIR}/include ${SENSOR_DIR}/include ${MAVLINK_DIR}/include ${LIVOX_SDK2_DIR}/include)
# 规划核心源文件整体纳入；传感器侧仅固定编译位姿缓冲与扫描装配，其余单元按需单列。
file(GLOB CORE_SRC ${CORE_DIR}/src/*.cpp)
set(SENSOR_SRC ${SENSOR_DIR}/src/pose_buffer.cpp ${SENSOR_DIR}/src/scan_assembler.cpp)
# mission_input 静态库：KML 任务解析，对外传递 LibXml2 的头文件与链接依赖。
add_library(mission_input src/mission_input.cpp)
target_include_directories(mission_input PRIVATE ${LIBXML2_INCLUDE_DIR})
target_link_libraries(mission_input PUBLIC ${LIBXML2_LIBRARIES})
# Livox SDK2 作为子工程编译；EXCLUDE_FROM_ALL 避免无引用时连带构建其样例与工具。
add_subdirectory(${LIVOX_SDK2_DIR}/sdk_core ${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2 EXCLUDE_FROM_ALL)
# 机载主程序：串口/MAVLink、EGO 规划适配、配置加载与 Livox 点云链路合并为单一可执行文件。
add_executable(onboard_main src/onboard_main.cpp src/error_log.cpp src/offboard_pilot.cpp src/ego_planner_adapter.cpp src/serial_port.cpp src/config_file.cpp src/livox_source.cpp ${CORE_SRC} ${SENSOR_SRC} ${SENSOR_DIR}/src/livox_parser.cpp ${SENSOR_DIR}/src/frame_assembler.cpp)
target_include_directories(onboard_main PRIVATE ${LIVOX_SDK2_DIR}/3rdparty ${LIVOX_SDK2_DIR}/sdk_core)
# livox_lidar_sdk_static 为 SDK2 的静态库目标；pthread 供串口与雷达接收线程使用。
target_link_libraries(onboard_main mission_input yaml-cpp pthread livox_lidar_sdk_static)

# 可选测试开关：默认 OFF，不进入装机产物；自动化冒烟测试用 -DBUILD_TESTING=ON 打开。
option(BUILD_TESTING "Build task-ingest and serial smoke tests" OFF)
# 注册 CTest，便于一次运行全部自检用例。
include(CTest)
if(BUILD_TESTING)
  include_directories(test)
  # 任务收件箱自检：KML 交付的读取与解析逻辑，不涉及飞行。
  add_executable(mission_inbox_selftest test/mission_inbox_selftest.cpp)
  target_link_libraries(mission_inbox_selftest mission_input)
  add_test(NAME mission_inbox_selftest COMMAND mission_inbox_selftest)
  # KML 解析冒烟：--prepare-only 不连飞控、不启动规划。
  add_test(NAME kml_prepare_only COMMAND onboard_main --prepare-only --kml ${CMAKE_CURRENT_SOURCE_DIR}/test/data/short_route.kml --config ${CMAKE_CURRENT_SOURCE_DIR}/config/onboard.yaml)
  # 装有 Python3 时才注册端到端用例（以子进程驱动主程序）。
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    # 运行中任务投递：模拟写入 KML，校验读取与装载流程。
    add_test(NAME mission_inbox_process_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mission_inbox_process_test.py $<TARGET_FILE:onboard_main> ${CMAKE_CURRENT_SOURCE_DIR}/config/onboard.yaml ${CMAKE_CURRENT_SOURCE_DIR}/test/data/short_route.kml)
    set_tests_properties(mission_inbox_process_test PROPERTIES TIMEOUT 30)
  endif()

  # Exercise the real serial transport on a pseudo terminal, including disconnect.
  # 用伪终端模拟真实串口并覆盖断链场景，不访问硬件设备。
  add_executable(serial_port_selftest test/serial_port_selftest.cpp src/serial_port.cpp)
  target_include_directories(serial_port_selftest PRIVATE src)
  target_link_libraries(serial_port_selftest pthread util)
  add_test(NAME serial_port_selftest COMMAND serial_port_selftest)
  set_tests_properties(serial_port_selftest PROPERTIES TIMEOUT 5)
endif()
