# cpp/server/CMakeLists.txt
# 仿真 LiDAR TCP 服务器模块

# ---- 服务器静态库 ----
add_library(lidar_server STATIC
    src/TcpServer.cpp
    src/SimDevice.cpp
    src/DevicePlaybackTiming.cpp
)

target_include_directories(lidar_server PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar_server PUBLIC
    lidar_protocol
    lidar_core
)

# 服务器需要线程库
find_package(Threads REQUIRED)
target_link_libraries(lidar_server PUBLIC Threads::Threads)

# Windows 需要链接 ws2_32
if(WIN32)
    target_link_libraries(lidar_server PUBLIC ws2_32)
endif()

# ---- 服务器可执行文件 ----
add_executable(lidar_sim_server
    apps/LidarSimServer.cpp
)

target_link_libraries(lidar_sim_server PRIVATE
    lidar_server
    lidar_protocol
    lidar_core
)
