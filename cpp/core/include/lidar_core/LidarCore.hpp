/**
 * @file LidarCore.hpp
 * @brief YLJ5 服务端、协议和客户端共享的核心 API 入口。
 *
 * 实现命名空间沿用 `lidar_demo`，设备、协议和客户端统一通过 `lidar_core` 别名访问；
 * 公共面只保留 YLJ5 正演、实时处理和 JSON 类型。
 */
#pragma once

#include "LidarDemo/LidarDemo.hpp"

namespace lidar_core = lidar_demo;
