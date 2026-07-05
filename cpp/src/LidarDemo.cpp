/**
 * @file LidarDemo.cpp
 * @brief Aggregate implementation unit for the atmospheric LiDAR particulate monitoring demo.
 */
#include "LidarDemo/LidarDemo.hpp"

// ============================================================================
// 标准库头文件
// ============================================================================
#include <algorithm>   ///< std::sort / std::min / std::max / std::all_of 等
#include <chrono>      ///< 性能计时（latency_ms）
#include <cmath>       ///< 数学函数：sin/cos/exp/sqrt/fmod 等
#include <cctype>      ///< isspace / isdigit 等字符判定
#include <cstdint>     ///< std::uint8_t 等定宽整型
#include <cstdio>      ///< snprintf 等
#include <ctime>       ///< 时间戳处理（mktime / localtime_r / strftime）
#include <fstream>     ///< 文件读写
#include <iomanip>     ///< put_time / get_time
#include <limits>      ///< 数值极限
#include <numbers>     ///< C++20 数学常量 pi_v
#include <numeric>     ///< std::accumulate 等
#include <queue>       ///< 热点检测连通域分析用的 BFS 队列
#include <random>      ///< 仿真用的 std::mt19937 正态分布
#include <set>         ///< 去重排序容器
#include <sstream>     ///< 字符串流（JSON 序列化、URL 拼接等）
#include <stdexcept>   ///< 异常类

// ============================================================================
// 平台相关的网络库：Windows 用 WinHTTP，其它平台（Linux/macOS）用 libcurl
// ============================================================================
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX            ///< 阻止 windows.h 定义 min/max 宏
#endif
#include <windows.h>
#include <winhttp.h>
#else
#include <curl/curl.h>
#endif

// NetCDF C API（仅在启用真实 Cloudnet 数据读取时需要）
#if defined(LIDAR_DEMO_HAS_NETCDF)
#include <netcdf.h>
#endif

namespace lidar_demo {

namespace {

#include "LidarDemo/MathUtilities.hpp"
#include "LidarDemo/NetcdfReader.hpp"
#include "LidarDemo/GroundRecordUtilities.hpp"
#include "LidarDemo/HttpDownloader.hpp"
#include "LidarDemo/SerializationAndResultTypes.hpp"
#include "LidarDemo/SimulationPhysics.hpp"
#include "LidarDemo/SimulatedCampaign.hpp"
#include "LidarDemo/CloudnetCampaign.hpp"
#include "LidarDemo/ProcessingAlgorithms.hpp"
#include "LidarDemo/HotspotDetection.hpp"
#include "LidarDemo/MetricsAndFailureCases.hpp"
#include "LidarDemo/PipelineRun.hpp"
#include "LidarDemo/DemoPayload.hpp"
#include "LidarDemo/PublicHelperUtilities.hpp"

} // namespace

#include "LidarDemo/DataFetchApi.hpp"
#include "LidarDemo/ConfigParser.hpp"
#include "LidarDemo/EndToEndApi.hpp"
#include "LidarDemo/DashboardApi.hpp"
#include "LidarDemo/ProcessingStepApi.hpp"

} // namespace lidar_demo
