# ============================================================================
# cpp/protocol/CMakeLists.txt
#
# lidar_protocol —— 传输层帧编解码库（JSON 行协议）。
# ============================================================================

# 查找已有的 lidar_core 目标
if(NOT TARGET lidar_core)
    message(FATAL_ERROR "lidar_core target must be defined before adding lidar_protocol")
endif()

add_library(lidar_protocol STATIC
    src/WireFormat.cpp
)

target_include_directories(lidar_protocol
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar_protocol
    PUBLIC
        lidar_core
)
