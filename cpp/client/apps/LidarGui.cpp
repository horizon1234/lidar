/**
 * @file LidarGui.cpp
 * @brief Linux Qt GUI 客户端入口。
 */
#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QTimer>

#include "lidar_client/MainWindow.hpp"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("YLJ5 LiDAR Monitor"));
    app.setApplicationVersion(QStringLiteral("1.0"));
    app.setOrganizationName(QStringLiteral("LiDAR"));
    app.setStyle(QStringLiteral("Fusion"));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("YLJ5 大气颗粒物激光雷达 Linux 监控客户端"));
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption host_option(
        QStringList{QStringLiteral("H"), QStringLiteral("host")},
        QStringLiteral("设备服务地址"), QStringLiteral("host"), QStringLiteral("127.0.0.1"));
    const QCommandLineOption port_option(
        QStringList{QStringLiteral("p"), QStringLiteral("port")},
        QStringLiteral("设备服务端口"), QStringLiteral("port"), QStringLiteral("19850"));
    const QCommandLineOption connect_option(
        QStringList{QStringLiteral("connect")}, QStringLiteral("窗口启动后自动连接"));
    const QCommandLineOption screenshot_option(
        QStringList{QStringLiteral("screenshot")},
        QStringLiteral("将主窗口保存为 PNG 后退出"), QStringLiteral("path"));
    parser.addOption(host_option);
    parser.addOption(port_option);
    parser.addOption(connect_option);
    parser.addOption(screenshot_option);
    parser.process(app);

    bool port_ok = false;
    const uint port_value = parser.value(port_option).toUInt(&port_ok);
    const quint16 port = port_ok && port_value > 0 && port_value <= 65535
        ? static_cast<quint16>(port_value)
        : static_cast<quint16>(19850);

    app.setStyleSheet(QStringLiteral(R"(
        QMainWindow, QWidget { color: #23313a; background: #f4f6f7; }
        QToolBar { background: #ffffff; border-bottom: 1px solid #ccd4d8; spacing: 5px; padding: 5px; }
        QToolButton { padding: 5px 7px; }
        QLineEdit, QSpinBox { background: #ffffff; border: 1px solid #b9c4c9; padding: 4px; }
        QFrame#statusBand { background: #ffffff; border: 1px solid #d5dde0; }
        QLabel#statusCaption { color: #65747c; font-size: 11px; }
        QLabel#statusValue { color: #1f3038; font-weight: 600; }
        QLabel#statusValue[validCalibration="true"] { color: #16715f; }
        QTabWidget::pane { border: 1px solid #ccd4d8; background: #ffffff; }
        QTabBar::tab { background: #e8edef; padding: 7px 16px; }
        QTabBar::tab:selected { background: #ffffff; color: #126d78; }
        QTableWidget, QTextEdit { background: #ffffff; border: 1px solid #ccd4d8; }
        QHeaderView::section { background: #e8edef; border: 0; border-right: 1px solid #d4dcdf; padding: 6px; }
        QStatusBar { background: #ffffff; border-top: 1px solid #ccd4d8; }
    )"));

    lidar_client::MainWindow window;
    window.set_endpoint(parser.value(host_option), port);
    window.show();

    if (parser.isSet(connect_option)) {
        QTimer::singleShot(0, &window, &lidar_client::MainWindow::connect_to_configured_server);
    }
    if (parser.isSet(screenshot_option)) {
        const QString path = parser.value(screenshot_option);
        QTimer::singleShot(1000, &app, [&app, &window, path]() {
            const bool saved = window.grab().save(path, "PNG");
            app.exit(saved ? 0 : 2);
        });
    }
    return app.exec();
}
