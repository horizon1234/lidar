cmake_minimum_required(VERSION 3.20)

project(ylj5_lidar_digital_twin LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "YLJ5 digital twin supports Linux only")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Shared logging used by every runtime module.  It is added first so its
# public target can be propagated transitively through lidar_core.
add_subdirectory(cpp/Log)

# YLJ5 服务端、协议和客户端共享的 JSON、四通道正演及实时处理算法。
add_library(lidar_core STATIC
    cpp/src/JsonEngine.cpp
    cpp/src/LidarDemo.cpp
)

target_include_directories(lidar_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/core/include
)

target_link_libraries(lidar_core PUBLIC lidar_log)

add_subdirectory(cpp/protocol)
add_subdirectory(cpp/server)
add_subdirectory(cpp/client)

# 第 19 章的三个纯 C++ 教学算例独立于设备运行时，继续作为知识学习材料保留。
add_executable(lidar_example_1d_ray cpp/examples/Example1dRay.cpp)
add_executable(lidar_example_2d_ppi cpp/examples/Example2dPpi.cpp)
add_executable(lidar_example_3d_rhi cpp/examples/Example3dRhi.cpp)

enable_testing()

add_executable(lidar_device_playback_timing_test cpp/tests/TestDevicePlaybackTiming.cpp)
target_link_libraries(lidar_device_playback_timing_test PRIVATE lidar_server)
add_test(NAME lidar_device_playback_timing_test COMMAND lidar_device_playback_timing_test)

add_executable(lidar_client_device_telemetry_test cpp/tests/TestClientDeviceTelemetry.cpp)
target_link_libraries(lidar_client_device_telemetry_test PRIVATE lidar_client)
add_test(NAME lidar_client_device_telemetry_test COMMAND lidar_client_device_telemetry_test)

add_executable(lidar_client_frame_processor_test cpp/tests/TestClientFrameProcessor.cpp)
target_link_libraries(lidar_client_frame_processor_test PRIVATE lidar_client)
add_test(NAME lidar_client_frame_processor_test COMMAND lidar_client_frame_processor_test)

add_executable(lidar_client_sim_device_pipeline_test cpp/tests/TestClientSimDevicePipeline.cpp)
target_link_libraries(lidar_client_sim_device_pipeline_test PRIVATE lidar_client lidar_server)
add_test(NAME lidar_client_sim_device_pipeline_test COMMAND lidar_client_sim_device_pipeline_test)

add_executable(lidar_sim_device_ylj5_test cpp/tests/TestSimDeviceYlj5.cpp)
target_link_libraries(lidar_sim_device_ylj5_test PRIVATE lidar_server)
add_test(NAME lidar_sim_device_ylj5_test COMMAND lidar_sim_device_ylj5_test)

add_executable(lidar_log_test cpp/tests/TestLog.cpp)
target_link_libraries(lidar_log_test PRIVATE lidar_log)
add_test(NAME lidar_log_test COMMAND lidar_log_test)
