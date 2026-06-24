# ============================================================================
# cpp/core/CMakeLists.txt
#
# lidar_core 共享核心库 —— 三层架构的数据/算法基石。
#
# 设计：lidar_core 是 lidar_demo 的命名空间别名（见 lidar_core.hpp），
# 因此本 CMake 目标只是一个 INTERFACE 库，把 include 路径和 lidar_demo
# 依赖传递给上层模块（protocol / server / client）。
# ============================================================================

# 查找已有的 lidar_demo 目标（由父目录的 CMakeLists.txt 定义）
if(NOT TARGET lidar_demo)
    message(FATAL_ERROR "lidar_demo target must be defined before adding lidar_core")
endif()

add_library(lidar_core INTERFACE)

# 核心头文件目录
target_include_directories(lidar_core
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# lidar_core 依赖 lidar_demo 的全部实现
target_link_libraries(lidar_core
    INTERFACE
        lidar_demo
)
