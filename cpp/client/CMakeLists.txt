# cpp/client/CMakeLists.txt
# 主控客户端模块（帧处理 + TCP 客户端）

# ---- 客户端静态库 ----
add_library(lidar_client STATIC
    src/tcp_client.cpp
    src/frame_processor.cpp
    # 闭环管理组件
    src/alarm_state_machine.cpp
    src/hotspot_tracker.cpp
    src/disposal_linkage.cpp
    src/report_generator.cpp
)

target_include_directories(lidar_client PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar_client PUBLIC
    lidar_protocol
    lidar_core
)

# 客户端需要线程库
find_package(Threads REQUIRED)
target_link_libraries(lidar_client PUBLIC Threads::Threads)

# Windows 需要链接 ws2_32
if(WIN32)
    target_link_libraries(lidar_client PUBLIC ws2_32)
endif()

# ---- 客户端可执行文件（无头模式）----
add_executable(lidar_control_client
    apps/main.cpp
)

target_link_libraries(lidar_control_client PRIVATE
    lidar_client
    lidar_protocol
    lidar_core
)

# ---- Qt UI（可选，需要 LIDAR_ENABLE_QT）----
option(LIDAR_ENABLE_QT "启用 Qt6 GUI 界面" OFF)
if(LIDAR_ENABLE_QT)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)

    find_package(Qt6 COMPONENTS Widgets QUIET)
    if(Qt6_FOUND)
        message(STATUS "Qt6 Widgets 找到，启用 GUI 客户端")

        # 将 Qt UI 源文件加入客户端库
        target_sources(lidar_client PRIVATE
            src/ppi_widget.cpp
            src/alarm_panel.cpp
            src/main_window.cpp
        )

        target_compile_definitions(lidar_client PUBLIC LIDAR_ENABLE_QT)
        target_link_libraries(lidar_client PUBLIC Qt6::Widgets)

        # GUI 可执行文件
        add_executable(lidar_gui
            apps/lidar_gui.cpp
        )
        target_link_libraries(lidar_gui PRIVATE
            lidar_client
            lidar_protocol
            lidar_core
        )
    else()
        message(WARNING "LIDAR_ENABLE_QT 已启用但未找到 Qt6 Widgets，跳过 GUI")
    endif()
endif()
