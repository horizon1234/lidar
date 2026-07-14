# Linux YLJ5 图形客户端。

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "YLJ5 client supports Linux only")
endif()

# 开发机 Qt 安装在 /opt。允许调用方用 LIDAR_QT_ROOT 覆盖；未指定时优先使用版本号
# 最高的 /opt/Qt6*/<version>/gcc_64，避免依赖用户 shell 中的环境变量。
set(LIDAR_QT_ROOT "" CACHE PATH "Qt 6 gcc_64 installation root")
if(LIDAR_QT_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${LIDAR_QT_ROOT}")
else()
    file(GLOB _lidar_qt_roots LIST_DIRECTORIES true "/opt/Qt6*/6.*/gcc_64")
    if(_lidar_qt_roots)
        list(SORT _lidar_qt_roots COMPARE NATURAL ORDER DESCENDING)
        list(GET _lidar_qt_roots 0 _lidar_qt_root)
        list(PREPEND CMAKE_PREFIX_PATH "${_lidar_qt_root}")
        message(STATUS "YLJ5 client Qt root: ${_lidar_qt_root}")
    endif()
endif()

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Network)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(lidar_client STATIC
    src/FrameProcessor.cpp
    src/DeviceStatusModel.cpp
    src/ScanCycleMonitor.cpp
    src/DisplaySnapshot.cpp
    src/LidarClientWorker.cpp
    src/PpiWidget.cpp
    src/VerticalProfileWidget.cpp
    src/AlarmPanel.cpp
    src/MainWindow.cpp
    src/AlarmStateMachine.cpp
    src/HotspotTracker.cpp
    src/DisposalLinkage.cpp
    src/ReportGenerator.cpp
    include/lidar_client/LidarClientWorker.hpp
    include/lidar_client/DisplaySnapshot.hpp
    include/lidar_client/MainWindow.hpp
    include/lidar_client/PpiWidget.hpp
    include/lidar_client/VerticalProfileWidget.hpp
    include/lidar_client/AlarmPanel.hpp
)

target_include_directories(lidar_client PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar_client PUBLIC
    lidar_protocol
    lidar_core
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
)

target_compile_definitions(lidar_client PUBLIC
    LIDAR_CLIENT_LINUX_QT=1
)

add_executable(lidar_gui
    apps/LidarGui.cpp
)

target_link_libraries(lidar_gui PRIVATE
    lidar_client
)
