/**
 * @file LidarCore.hpp
 * @brief lidar_core —— 共享核心库的桥接头文件
 *
 * 本文件是三层架构重构（sim_server → control_client → UI）的核心库入口。
 *
 * 设计决策：
 *   原有 lidar_demo 命名空间中已经包含了完整、经过测试的全部算法实现
 *   （JSON 引擎、数学/时间工具、NetCDF 读取、HTTP 下载、前向仿真、L1
 *   预处理、Fernald 反演、湿度校正、ENU 转换、PM 标定、热点检测、质量
 *   评估、管线编排等），共 4500+ 行。为避免重复抽取和维护两套代码，
 *   lidar_core 命名空间直接作为 lidar_demo 的别名。
 *
 *   新架构中的协议层、服务端、客户端、闭环管理器、UI 层均通过
 *   `#include "lidar_core/LidarCore.hpp"` 获取全部核心类型与算法，
 *   使用 `lidar_core::` 命名空间前缀引用。
 *
 * 使用方式：
 *   #include "lidar_core/LidarCore.hpp"
 *   lidar_core::PipelineConfig config = ...;
 *   lidar_core::Json payload = lidar_core::run_end_to_end(config);
 */
#pragma once

#include "LidarDemo/LidarDemo.hpp"

namespace lidar_core = lidar_demo;
