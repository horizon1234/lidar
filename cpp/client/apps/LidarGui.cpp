/**
 * @file lidar_gui.cpp
 * @brief GUI 入口点：启动 Qt 主窗口。
 */
#ifdef LIDAR_ENABLE_QT

#include <QApplication>
#include "lidar_client/MainWindow.hpp"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("LiDAR PM Monitor");
    app.setOrganizationName("LiDAR");

    lidar_client::MainWindow window;
    window.show();

    return app.exec();
}

#endif // LIDAR_ENABLE_QT
