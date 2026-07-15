# Log - shared, dependency-free logging library.

add_library(lidar_log STATIC
    src/Logger.cpp
)

add_library(Log ALIAS lidar_log)

find_package(Threads REQUIRED)

target_include_directories(lidar_log
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(lidar_log PUBLIC cxx_std_20)

if(NOT WIN32)
    target_link_libraries(lidar_log PUBLIC Threads::Threads)
endif()
