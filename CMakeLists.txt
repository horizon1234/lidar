cmake_minimum_required(VERSION 3.20)

project(atmospheric_lidar_pollution_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(netCDF REQUIRED)
find_package(CURL REQUIRED)

# ---- 所有可执行文件统一输出到项目根目录的 bin/ ----
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_library(lidar_demo STATIC
    cpp/src/JsonEngine.cpp
    cpp/src/LidarDemo.cpp
)

target_include_directories(lidar_demo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${netCDF_INCLUDE_DIRS}
)

target_compile_definitions(lidar_demo PRIVATE LIDAR_DEMO_HAS_NETCDF=1)

target_link_libraries(lidar_demo PUBLIC
    ${netCDF_LIBRARIES}
    CURL::libcurl
)

if (WIN32)
    target_link_libraries(lidar_demo PUBLIC winhttp)
endif()

add_executable(lidar_run_batch cpp/apps/RunBatch.cpp)
target_link_libraries(lidar_run_batch PRIVATE lidar_demo)

add_executable(lidar_build_demo_assets cpp/apps/BuildDemoAssets.cpp)
target_link_libraries(lidar_build_demo_assets PRIVATE lidar_demo)

add_executable(lidar_api_server cpp/apps/ApiServer.cpp)
target_link_libraries(lidar_api_server PRIVATE lidar_demo)

if (WIN32)
    target_link_libraries(lidar_api_server PRIVATE ws2_32)
endif()

add_executable(lidar_fetch_public_ground_data cpp/apps/FetchPublicGroundData.cpp)
target_link_libraries(lidar_fetch_public_ground_data PRIVATE lidar_demo)

add_executable(lidar_fetch_cloudnet_public_sample cpp/apps/FetchCloudnetPublicSample.cpp)
target_link_libraries(lidar_fetch_cloudnet_public_sample PRIVATE lidar_demo)

# ---- 三层架构：核心库 → 协议层 → 仿真服务器 / 主控客户端 ----
add_subdirectory(cpp/core)
add_subdirectory(cpp/protocol)
add_subdirectory(cpp/server)
add_subdirectory(cpp/client)

# ---- 第 19 章教学算例 (cpp/examples/) ----
# 这些算例是纯 C++ 项目, 只依赖 STL, 不依赖 lidar_demo 库或 NetCDF,
# 这样学习者只看算法本身, 不被框架代码绕进去。
add_executable(lidar_example_1d_ray cpp/examples/Example1dRay.cpp)
add_executable(lidar_example_2d_ppi cpp/examples/Example2dPpi.cpp)
add_executable(lidar_example_3d_rhi cpp/examples/Example3dRhi.cpp)
add_executable(lidar_run_full_demo  cpp/examples/RunFullDemo.cpp)
# run_full_demo 调用 lidar_demo 库 + 子进程串联 stage 1-3, 所以需要链接库
target_link_libraries(lidar_run_full_demo PRIVATE lidar_demo)

enable_testing()

add_executable(lidar_pipeline_test cpp/tests/TestPipeline.cpp)
target_link_libraries(lidar_pipeline_test PRIVATE lidar_demo)

add_test(
    NAME lidar_pipeline_test
    COMMAND lidar_pipeline_test ${CMAKE_CURRENT_SOURCE_DIR}/configs/DefaultPipeline.json
)

# 闭环管理组件测试
add_executable(lidar_closed_loop_test cpp/tests/TestClosedLoop.cpp)
target_link_libraries(lidar_closed_loop_test PRIVATE lidar_client lidar_core)

add_test(
    NAME lidar_closed_loop_test
    COMMAND lidar_closed_loop_test
)

add_executable(lidar_device_playback_timing_test cpp/tests/TestDevicePlaybackTiming.cpp)
target_link_libraries(lidar_device_playback_timing_test PRIVATE lidar_server lidar_protocol lidar_core)

add_test(
    NAME lidar_device_playback_timing_test
    COMMAND lidar_device_playback_timing_test
)

add_executable(lidar_client_device_telemetry_test cpp/tests/TestClientDeviceTelemetry.cpp)
target_link_libraries(lidar_client_device_telemetry_test PRIVATE lidar_client lidar_protocol lidar_core)

add_test(
    NAME lidar_client_device_telemetry_test
    COMMAND lidar_client_device_telemetry_test
)
