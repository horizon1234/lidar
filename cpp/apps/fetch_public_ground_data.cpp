/**
 * @file fetch_public_ground_data.cpp
 * @brief 地面公开数据抓取 CLI：从 Open-Meteo 等公开源获取地面 PM/气象数据。
 *
 * 该工具是 lidar_demo 数据采集阶段的命令行封装：
 * - 接受经纬度、时间窗、时区与输出前缀等参数（默认定位北京）；
 * - 调用 lidar_demo::fetch_public_ground_data 抓取地面观测与气象数据；
 * - 把抓取结果（CSV/JSON/manifest）写入 data/public 下；
 * - 将 manifest 以缩进 JSON 形式打印到标准输出，供下游流水线使用。
 */

#include "lidar_demo/lidar_demo.hpp"

#include <iostream>

/**
 * @brief 地面数据抓取主入口：解析地理与时间参数并触发抓取。
 *
 * 支持以下命令行参数（顺序无关，均为“键 + 值”形式）：
 *   --latitude <double>    纬度（默认 39.9042，即北京）
 *   --longitude <double>   经度（默认 116.4074，即北京）
 *   --start-date <YYYY-MM-DD> 抓取起始日期（默认 2026-05-30）
 *   --end-date <YYYY-MM-DD>   抓取结束日期（默认 2026-05-31）
 *   --timezone <tz>        IANA 时区名（默认 Asia/Shanghai）
 *   --prefix <name>        文件名前缀（默认 open_meteo_beijing）
 *   --output-root <dir>    输出根目录（默认当前目录 "."）
 *
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组。
 * @return int 进程返回码：0 表示成功，1 表示异常。
 */
int main(int argc, char** argv) {
    try {
        // === 默认参数：定位北京并使用固定时间窗，便于开箱即用 ===
        double latitude = 39.9042; ///< 纬度
        double longitude = 116.4074; ///< 经度
        std::string start_date = "2026-05-30"; ///< 时间窗起始（YYYY-MM-DD）
        std::string end_date = "2026-05-31"; ///< 时间窗结束（YYYY-MM-DD）
        std::string timezone = "Asia/Shanghai"; ///< IANA 时区
        std::string prefix = "open_meteo_beijing"; ///< 输出文件名前缀
        std::filesystem::path output_root = "."; ///< 输出根目录

        // === 命令行参数解析：经纬度用 std::stod 转双精度 ===
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--latitude" && index + 1 < argc) {
                latitude = std::stod(argv[++index]); // 字符串转 double
            } else if (argument == "--longitude" && index + 1 < argc) {
                longitude = std::stod(argv[++index]);
            } else if (argument == "--start-date" && index + 1 < argc) {
                start_date = argv[++index];
            } else if (argument == "--end-date" && index + 1 < argc) {
                end_date = argv[++index];
            } else if (argument == "--timezone" && index + 1 < argc) {
                timezone = argv[++index];
            } else if (argument == "--prefix" && index + 1 < argc) {
                prefix = argv[++index];
            } else if (argument == "--output-root" && index + 1 < argc) {
                output_root = argv[++index];
            }
        }

        // === 数据抓取：调用公开数据采集 API 并写入 data/public 子目录 ===
        lidar_demo::Json manifest = lidar_demo::fetch_public_ground_data(
            latitude,
            longitude,
            start_date,
            end_date,
            timezone,
            output_root / "data" / "public", // 公开数据统一落在 data/public 下
            prefix
        );
        std::cout << lidar_demo::dump_json(manifest, 2) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        // 统一异常处理：错误信息写入 stderr，返回码 1
        std::cerr << error.what() << std::endl;
        return 1;
    }
}