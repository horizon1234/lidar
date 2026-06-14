cmake_minimum_required(VERSION 3.20)

project(atmospheric_lidar_pollution_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(NetCDF REQUIRED)

add_library(lidar_demo STATIC
    cpp/src/lidar_demo.cpp
)

target_include_directories(lidar_demo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${NetCDF_INCLUDE_DIRS}
)

target_compile_definitions(lidar_demo PRIVATE LIDAR_DEMO_HAS_NETCDF=1)

target_link_libraries(lidar_demo PUBLIC
    ${NetCDF_LIBRARIES}
)

if (WIN32)
    target_link_libraries(lidar_demo PUBLIC winhttp)
endif()

add_executable(lidar_run_batch cpp/apps/run_batch.cpp)
target_link_libraries(lidar_run_batch PRIVATE lidar_demo)

add_executable(lidar_build_demo_assets cpp/apps/build_demo_assets.cpp)
target_link_libraries(lidar_build_demo_assets PRIVATE lidar_demo)

add_executable(lidar_api_server cpp/apps/api_server.cpp)
target_link_libraries(lidar_api_server PRIVATE lidar_demo)

if (WIN32)
    target_link_libraries(lidar_api_server PRIVATE ws2_32)
endif()

add_executable(lidar_fetch_public_ground_data cpp/apps/fetch_public_ground_data.cpp)
target_link_libraries(lidar_fetch_public_ground_data PRIVATE lidar_demo)

add_executable(lidar_fetch_cloudnet_public_sample cpp/apps/fetch_cloudnet_public_sample.cpp)
target_link_libraries(lidar_fetch_cloudnet_public_sample PRIVATE lidar_demo)

# ---- 第 19 章教学算例 (cpp/examples/) ----
# 这些算例是纯 C++ 项目, 只依赖 STL, 不依赖 lidar_demo 库或 NetCDF,
# 这样学习者只看算法本身, 不被框架代码绕进去。
add_executable(lidar_example_1d_ray cpp/examples/example_1d_ray.cpp)
add_executable(lidar_example_2d_ppi cpp/examples/example_2d_ppi.cpp)
add_executable(lidar_example_3d_rhi cpp/examples/example_3d_rhi.cpp)
add_executable(lidar_run_full_demo  cpp/examples/run_full_demo.cpp)
# run_full_demo 调用 lidar_demo 库 + 子进程串联 stage 1-3, 所以需要链接库
target_link_libraries(lidar_run_full_demo PRIVATE lidar_demo)

enable_testing()

add_executable(lidar_pipeline_test cpp/tests/test_pipeline.cpp)
target_link_libraries(lidar_pipeline_test PRIVATE lidar_demo)

add_test(
    NAME lidar_pipeline_test
    COMMAND lidar_pipeline_test ${CMAKE_CURRENT_SOURCE_DIR}/configs/default_pipeline.json
)
