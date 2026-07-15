/**
 * @file FrameProcessor.cpp
 * @brief YLJ5 实时设备处理链实现。
 *
 * 一条 lidar_raw 射线在本文件中的处理顺序为：
 *
 * 1. 校正距离零点并校验所有输入数组；
 * 2. 统一累计计数/每脉冲平均计数的单位；
 * 3. 分通道估计背景，校正饱和、光子计数死时间和 afterpulse；
 * 4. 扣背景、除通道增益和 overlap，生成逐 bin SNR 与质量位；
 * 5. 在共同有效区估算近远场比例，并按距离连续拼接两路平行偏振信号；
 * 6. 做激光能量归一化和距离平方校正，得到反演输入；
 * 7. 执行 Fernald/Klett、湿度修正、PM 标定门控和 ENU 投影；
 * 8. 在 heartbeat 或时间戳变化时汇总一个不可变 StepResult。
 *
 * 这里最重要的约束是“无效值不能伪装成零”。几何盲区、饱和、死时间不可逆、低 SNR
 * 等情况使用 bin_quality 记录并在数值数组中保留 NaN，下游显示和热点算法必须显式跳过。
 */
#include "lidar_client/FrameProcessor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "lidar_log/Logger.hpp"

namespace lidar_client {

namespace {

constexpr std::size_t DebugArrayPreviewCount = 12;

/**
 * @brief 递归构造用于日志的 JSON 预览，不改变算法实际消费的 JSON。
 *
 * 回波帧中的 ranges_m、raw_counts、overlap 等数组可能包含数千个 bin。日志只保留每个
 * 数组的前 12 项，并追加一个 JSON 对象说明省略了多少项；短数组和普通对象字段完整保留。
 */
lidar_core::Json make_debug_json_preview(const lidar_core::Json& value) {
    if (value.is_array()) {
        const auto& items = value.array_items();
        lidar_core::Json::Array preview;
        const std::size_t visible_count = std::min(
            items.size(), DebugArrayPreviewCount);
        preview.reserve(visible_count + (items.size() > visible_count ? 1 : 0));
        for (std::size_t index = 0; index < visible_count; ++index) {
            preview.push_back(make_debug_json_preview(items[index]));
        }
        if (items.size() > visible_count) {
            preview.emplace_back(lidar_core::Json::Object{
                {"_truncated", true},
                {"omitted_count", static_cast<int>(items.size() - visible_count)},
            });
        }
        return lidar_core::Json(std::move(preview));
    }

    if (value.is_object()) {
        lidar_core::Json::Object preview;
        for (const auto& [key, item] : value.object_items()) {
            preview.emplace(key, make_debug_json_preview(item));
        }
        return lidar_core::Json(std::move(preview));
    }

    return value;
}

/** @brief 把协议 Frame 组合成带 type/timestamp 的缩进 JSON 日志文本。 */
std::string make_debug_frame_json(const lidar_protocol::Frame& frame) {
    lidar_core::Json::Object root{
        {"type", lidar_protocol::frame_type_to_string(frame.type)},
        {"timestamp", frame.timestamp},
    };
    if (frame.payload.is_object()) {
        for (const auto& [key, value] : frame.payload.object_items()) {
            root[key] = make_debug_json_preview(value);
        }
    } else {
        root["payload"] = make_debug_json_preview(frame.payload);
    }
    return lidar_core::dump_json(lidar_core::Json(std::move(root)), 2);
}

/**
 * @brief 从 JSON 对象读取数值字段。
 *
 * 协议适配层允许部分元数据暂时缺失，因此这里只负责“类型正确则读取”，否则返回调用方
 * 提供的 fallback。物理上不可缺失的字段不能只依赖这个回退值，后续还会由处理链单独
 * 检查，例如激光能量缺失会把所有反演 bin 标成 invalid_laser_energy。
 */
double json_number(const lidar_core::Json& json, const char* key, double fallback) {
    return json.contains(key) && json.at(key).is_number()
        ? json.at(key).number_value()
        : fallback;
}

/** @brief 从 JSON 对象读取字符串字段；缺失或类型错误时返回空字符串。 */
std::string json_string(const lidar_core::Json& json, const char* key) {
    return json.contains(key) && json.at(key).is_string()
        ? json.at(key).string_value()
        : "";
}

/**
 * @brief 计算样本中位数，不修改调用方持有的原数组。
 *
 * 背景包络和近远场比例都可能混入云、强散射体等离群点。中位数比算术平均更不容易被
 * 少量极端值拖动。nth_element 只做部分排序，复杂度和内存开销均适合 5334-bin 实时链。
 */
double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2;
    // nth_element 保证 middle 位置就是排序后的中间元素，但不要求其余元素完全有序。
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    double result = values[middle];
    if (values.size() % 2 == 0) {
        // 偶数样本的中位数是下半区最大值与上半区最小值的平均。
        auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        result = (*lower + result) * 0.5;
    }
    return result;
}

/**
 * @brief 距离相关背景的线性模型 B(r) = intercept + slope_per_m * r。
 *
 * 理想光子计数背景近似常数，但日光、模拟前端基线和距离门电子学响应可能引入缓慢斜率。
 * measured=false 表示没有足够的远端样本完成拟合，此时 intercept 来自设备元数据。
 */
struct BackgroundModel {
    double intercept = 0.0;
    double slope_per_m = 0.0;
    bool measured = false;

    /** @brief 计算指定距离的背景，并把物理上无意义的负预测值截到 0。 */
    double at(double range_m) const {
        return std::max(intercept + slope_per_m * range_m, 0.0);
    }
};

/**
 * @brief 对 (range_m, background_counts) 样本做普通最小二乘直线拟合。
 *
 * 上游 estimate_background 已经按块提取低四分位包络，这里拟合的不是全部原始回波，
 * 因而无需再次做复杂的鲁棒回归。返回值依次为截距和每米斜率。
 */
std::pair<double, double> fit_line(
    const std::vector<std::pair<double, double>>& samples) {
    if (samples.size() < 2) {
        return {samples.empty() ? 0.0 : samples.front().second, 0.0};
    }
    double x_mean = 0.0;
    double y_mean = 0.0;
    // 先计算样本中心，随后用中心化形式求协方差，数值稳定性优于直接展开正规方程。
    for (const auto& sample : samples) {
        x_mean += sample.first;
        y_mean += sample.second;
    }
    x_mean /= static_cast<double>(samples.size());
    y_mean /= static_cast<double>(samples.size());
    double covariance = 0.0;
    double variance = 0.0;
    for (const auto& sample : samples) {
        covariance += (sample.first - x_mean) * (sample.second - y_mean);
        variance += (sample.first - x_mean) * (sample.first - x_mean);
    }
    // 所有距离几乎相同时无法估计斜率，此时安全退化为常数背景。
    const double slope = variance > 1e-12 ? covariance / variance : 0.0;
    return {y_mean - slope * x_mean, slope};
}

/**
 * @brief 从远端候选区估计单通道背景曲线。
 *
 * @param channel 当前物理接收通道，raw_counts 与 ranges_m 对齐。
 * @param ranges_m 已应用距离零点修正的距离轴，单位 m。
 * @param raw_scale 把协议计数换算到“每脉冲平均计数”的比例。
 * @param config 背景候选区、最少样本数和尾段比例配置。
 *
 * 算法先保留 channel.background_counts 作为始终可用的元数据回退，然后只在
 * background_minimum_range_m 以外的尾段取样。尾段被分成多个小块，每块取计数低四分位
 * 点形成“背景下包络”，以减少薄云和远端气溶胶这种只会把信号抬高的污染。下包络点再
 * 拟合成缓慢变化的直线。若距离范围太短或有效样本不足，measured 保持 false。
 */
BackgroundModel estimate_background(
    const lidar_core::LidarChannel& channel,
    const std::vector<double>& ranges_m,
    double raw_scale,
    const ProcessorConfig& config) {
    BackgroundModel output;
    // 元数据背景是拟合失败时的保底值；同样要换算到统一的每脉冲计数单位。
    output.intercept = std::max(channel.background_counts * raw_scale, 0.0);
    if (channel.raw_counts.empty() || ranges_m.size() != channel.raw_counts.size()) {
        return output;
    }

    // 尾段长度同时受比例、最小 bin 数和最大计算量约束。
    const std::size_t minimum_bins = static_cast<std::size_t>(
        std::max(config.minimum_background_bins, 4));
    const std::size_t maximum_bins = static_cast<std::size_t>(
        std::max(config.maximum_background_bins, config.minimum_background_bins));
    const std::size_t requested = static_cast<std::size_t>(std::ceil(
        std::clamp(config.background_tail_fraction, 0.01, 0.50)
        * static_cast<double>(channel.raw_counts.size())));
    const std::size_t tail_count = std::min(
        channel.raw_counts.size(), std::clamp(requested, minimum_bins, maximum_bins));
    std::size_t begin = channel.raw_counts.size() - tail_count;
    // 即使尾段比例给出了更近的起点，也不允许进入配置的最小背景距离以内。
    const auto background_begin = std::lower_bound(
        ranges_m.begin(), ranges_m.end(), config.background_minimum_range_m);
    begin = std::max(begin, static_cast<std::size_t>(
        std::distance(ranges_m.begin(), background_begin)));
    if (channel.raw_counts.size() - begin < minimum_bins) {
        // 短量程测试或真实设备未提供足够背景窗时，使用设备报告的背景基线。
        return output;
    }

    // 每个小段取低四分位点，降低远端薄云或残余气溶胶对背景拟合的正偏差。
    const std::size_t block_size = std::max<std::size_t>(4, tail_count / 16);
    std::vector<std::pair<double, double>> lower_envelope;
    for (std::size_t block_begin = begin;
         block_begin < channel.raw_counts.size(); block_begin += block_size) {
        const std::size_t block_end = std::min(
            channel.raw_counts.size(), block_begin + block_size);
        std::vector<std::pair<double, double>> block;
        block.reserve(block_end - block_begin);
        for (std::size_t index = block_begin; index < block_end; ++index) {
            const double value = channel.raw_counts[index] * raw_scale;
            // 非有限或负计数不能进入统计，否则可能把整条拟合线污染为 NaN。
            if (std::isfinite(value) && std::isfinite(ranges_m[index]) && value >= 0.0) {
                block.emplace_back(value, ranges_m[index]);
            }
        }
        if (block.empty()) continue;
        // block 内 pair.first 是计数，按计数选择第 25% 分位；pair.second 才是距离。
        const std::size_t quartile = block.size() / 4;
        std::nth_element(
            block.begin(), block.begin() + static_cast<std::ptrdiff_t>(quartile), block.end(),
            [](const auto& left, const auto& right) { return left.first < right.first; });
        lower_envelope.emplace_back(block[quartile].second, block[quartile].first);
    }
    if (lower_envelope.size() < 2) {
        // 少于两个包络点无法辨识斜率，继续使用元数据常数背景。
        return output;
    }
    const auto [intercept, slope] = fit_line(lower_envelope);
    if (std::isfinite(intercept) && std::isfinite(slope)) {
        // 只有拟合参数均有限时才声明 measured，避免错误状态进入逐 bin 扣背景。
        output.intercept = intercept;
        output.slope_per_m = slope;
        output.measured = true;
    }
    return output;
}

/**
 * @brief 按望远镜路径和偏振分量查找物理通道。
 *
 * 不依赖 channel_id 的具体厂商命名，使未来 VendorProtocolAdapter 只需把 telescope 和
 * polarization 归一化为 near/far、parallel/perpendicular 即可复用处理链。
 */
const lidar_core::LidarChannel* find_channel(
    const lidar_core::LidarProfile& profile,
    const std::string& telescope,
    const std::string& polarization) {
    for (const auto& channel : profile.channels) {
        if (channel.telescope == telescope && channel.polarization == polarization) {
            return &channel;
        }
    }
    return nullptr;
}

/**
 * @brief 在任何数值算法运行前验证一条射线的结构和基本物理范围。
 *
 * 这里验证的是“能否安全进入算法”，不是最终科学质量。低 SNR、部分 overlap 等情况仍是
 * 合法输入，随后通过逐 bin 质量位降级；数组错位、非有限原始值或负增益则会导致整条射线
 * 被拒绝，因为继续计算会造成越界或无法解释的物理结果。
 */
void validate_profile(const lidar_core::LidarProfile& profile) {
    const std::size_t count = profile.ranges_m.size();
    // 主距离轴、兼容主通道和主 overlap 必须严格一一对应。
    if (count < 8 || profile.raw_counts.size() != count || profile.overlap.size() != count) {
        throw std::runtime_error("invalid lidar profile shape");
    }
    double previous = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < count; ++index) {
        // 严格递增是距离积分、插值、投影和背景区二分查找共同依赖的不变量。
        if (!std::isfinite(profile.ranges_m[index])
            || !std::isfinite(profile.raw_counts[index])
            || !std::isfinite(profile.overlap[index])
            || profile.overlap[index] < 0.0
            || profile.overlap[index] > 1.0
            || profile.ranges_m[index] <= previous) {
            throw std::runtime_error("invalid or non-monotonic lidar samples");
        }
        previous = profile.ranges_m[index];
    }
    if (!std::isfinite(profile.azimuth_deg) || !std::isfinite(profile.elevation_deg)
        || profile.azimuth_deg < 0.0 || profile.azimuth_deg >= 360.0
        || profile.elevation_deg < 0.0 || profile.elevation_deg > 180.0) {
        throw std::runtime_error("lidar pointing is outside public gimbal limits");
    }
    // 上游质量位可以缺失；一旦存在，就必须和距离轴完全对齐。
    if (!profile.device_bin_quality.empty()
        && profile.device_bin_quality.size() != count) {
        throw std::runtime_error("main-channel quality mask does not match range axis");
    }
    for (const auto& channel : profile.channels) {
        // 每个物理通道共享 profile.ranges_m，不重复携带独立距离轴。
        if (channel.raw_counts.size() != count || channel.overlap.size() != count) {
            throw std::runtime_error("receiver channel shape does not match range axis");
        }
        if (!channel.device_bin_quality.empty()
            && channel.device_bin_quality.size() != count) {
            throw std::runtime_error("receiver channel quality mask does not match range axis");
        }
        if (!std::isfinite(channel.relative_gain) || channel.relative_gain <= 0.0) {
            throw std::runtime_error("receiver channel gain is invalid");
        }
        for (std::size_t index = 0; index < count; ++index) {
            // overlap 是收光比例，物理定义域为 [0, 1]；0 合法但会在后续被 mask。
            if (!std::isfinite(channel.raw_counts[index])
                || !std::isfinite(channel.overlap[index])
                || channel.overlap[index] < 0.0
                || channel.overlap[index] > 1.0) {
                throw std::runtime_error("receiver channel contains invalid samples");
            }
        }
    }
}

/**
 * @brief 在设备帧未携带分子场时生成 532 nm 标准大气近似。
 *
 * Fernald 反演必须知道分子后向散射和分子消光。真实业务应使用同期温压廓线或数值天气
 * 模式；这里只用 8 km 指数尺度高度和 Rayleigh 的 lambda^-4 波长关系作为明确可追踪的
 * 回退。数值单位为 km^-1（后向散射另带 sr^-1），不能与按 m^-1 存储的外部数据混用。
 */
void fill_standard_molecular_reference(
    lidar_core::LidarProfile& profile,
    double site_altitude_m,
    double wavelength_nm,
    std::vector<std::string>& qc_flags) {
    const std::size_t count = profile.ranges_m.size();
    if (profile.molecular_backscatter.size() == count
        && profile.molecular_extinction.size() == count) {
        // 同时齐全才接受上游分子场；只缺一项时统一重建，避免两项来自不同气象假设。
        return;
    }

    profile.molecular_backscatter.assign(count, 0.0);
    profile.molecular_extinction.assign(count, 0.0);
    const double elevation_rad = profile.elevation_deg * std::acos(-1.0) / 180.0;
    // 532 nm 为参考波长；Rayleigh 散射强度近似与波长四次方成反比。
    const double rayleigh_scale = std::pow(532.0 / std::max(wavelength_nm, 1.0), 4.0);
    for (std::size_t index = 0; index < count; ++index) {
        // 斜程距离投影到高度：h = site_altitude + r * sin(elevation)。
        const double height_m = std::max(
            0.0,
            site_altitude_m + profile.ranges_m[index] * std::sin(elevation_rad));
        // 海平面 532 nm 分子消光约取 0.012 km^-1，并按 8 km 尺度高度指数衰减。
        const double molecular_extinction_per_km =
            0.012 * rayleigh_scale * std::exp(-height_m / 8000.0);
        profile.molecular_extinction[index] = molecular_extinction_per_km;
        // 固定分子消光/后向散射比 8 sr，即 beta_m = alpha_m / 8。
        profile.molecular_backscatter[index] = molecular_extinction_per_km / 8.0;
    }
    qc_flags.emplace_back("molecular-reference-standard-atmosphere");
}

/**
 * @brief 单个物理接收通道完成 L1 校正后的中间结果。
 *
 * corrected 已依次完成探测器非线性、afterpulse、背景、相对增益和 overlap 校正；无效
 * bin 为 NaN。snr 仍基于观测计数估计，因为反校正后的噪声并不再服从简单 Poisson 分布。
 * 三个 bool 用于生成廓线级 QC，说明本次处理实际采用了哪些数据来源或校正步骤。
 */
struct ChannelSignal {
    std::vector<double> corrected; ///< 探测器、背景、增益和 overlap 修正后的信号。
    std::vector<double> snr;       ///< 按输入计数单位和积分脉冲数估计的信噪比。
    std::vector<lidar_core::BinQualityMask> quality; ///< 每个距离门的失效/降级原因位集合。
    bool background_measured = false; ///< true 表示背景来自远端拟合，而非设备元数据回退。
    bool dead_time_corrected = false; ///< true 表示配置了并执行了光子计数死时间逆校正。
    bool afterpulse_corrected = false; ///< true 表示 afterpulse 核至少含一个正系数。
};

/**
 * @brief 对一条物理接收通道执行探测器和光学 L1 校正。
 *
 * @param channel 原始计数、相对增益、overlap 和可选上游质量位。
 * @param ranges_m 校正后的共享距离轴，单位 m。
 * @param raw_scale 原始计数到每脉冲平均计数的换算比例。
 * @param pulse_gain sqrt(integrated_pulses)，用于平均计数的 Poisson SNR 恢复。
 * @param calibration 接收机死时间、饱和、近场和 afterpulse 标定。
 * @param config 背景拟合和最低通道 SNR 配置。
 *
 * 处理顺序不能随意调整：死时间作用于“信号+背景+afterpulse”的总观测计数，必须先对
 * 总计数线性化；afterpulse 是前序距离门向当前门的卷积泄漏，在线性化之后递推扣除；
 * 随后才可扣背景并做增益/overlap 校正。低 overlap 不使用数值下限强行除法，而是 mask。
 */
ChannelSignal correct_channel(
    const lidar_core::LidarChannel& channel,
    const std::vector<double>& ranges_m,
    double raw_scale,
    double pulse_gain,
    const ReceiverCalibrationModel& calibration,
    const ProcessorConfig& config) {
    ChannelSignal output;
    output.corrected.resize(channel.raw_counts.size(), 0.0);
    output.snr.resize(channel.raw_counts.size(), 0.0);
    output.quality.resize(channel.raw_counts.size(), 0);
    // NaN 明确表示“不可用于计算”，与物理上可能成立的零回波严格区分。
    const double invalid = std::numeric_limits<double>::quiet_NaN();

    // 背景在探测器非线性校正前估计，因为原始远端样本也处于观测计数域。
    const BackgroundModel background = estimate_background(
        channel, ranges_m, raw_scale, config);
    output.background_measured = background.measured;
    // relative_gain 把各接收链统一到共同响应尺度；输入验证已保证它严格为正。
    const double receiver_gain = channel.relative_gain;

    // 在硬饱和点之前留出 guard，避免 ADC/计数器临近满量程时已经发生的非线性被忽略。
    const double saturation_limit = calibration.saturation_counts > 0.0
        ? calibration.saturation_counts
            * std::clamp(calibration.saturation_guard_fraction, 0.5, 1.0)
        : std::numeric_limits<double>::infinity();
    // dead_time_loss=0 表示模拟采集或显式关闭；负标定值不具物理意义，安全截到 0。
    const double dead_time_loss = std::max(calibration.dead_time_loss_per_count, 0.0);
    const double maximum_occupancy = std::clamp(
        calibration.maximum_dead_time_occupancy, 0.1, 0.95);
    output.dead_time_corrected = dead_time_loss > 0.0;
    output.afterpulse_corrected = std::any_of(
        calibration.afterpulse_kernel.begin(), calibration.afterpulse_kernel.end(),
        [](double value) { return std::isfinite(value) && value > 0.0; });

    // afterpulse 反卷积是递推过程，需要保存已经重建的无 afterpulse 总计数和背景序列。
    std::vector<double> primary_total(channel.raw_counts.size(), 0.0);
    std::vector<double> primary_background(channel.raw_counts.size(), 0.0);
    // 若前序 bin 饱和或死时间不可逆，其泄漏量也不可精确恢复，后继 bin 必须同步降级。
    std::vector<bool> reconstruction_valid(channel.raw_counts.size(), true);
    for (std::size_t index = 0; index < channel.raw_counts.size(); ++index) {
        auto& quality = output.quality[index];

        // 设备固件或 VendorProtocolAdapter 给出的质量位优先保留，客户端只在其上追加原因。
        if (channel.device_bin_quality.size() == channel.raw_counts.size()) {
            quality |= channel.device_bin_quality[index];
        }

        // integrated_counts 会在进入本函数前通过 raw_scale 除以积分脉冲数。
        const double raw = channel.raw_counts[index] * raw_scale;
        const double observed_background = background.at(ranges_m[index]);
        const double overlap = channel.overlap[index];

        // 第一组质量门控只判断输入本身和静态标定边界，不进行任何反演。
        if (!std::isfinite(raw) || !std::isfinite(observed_background)
            || !std::isfinite(overlap)) {
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::non_finite_input);
        }
        if (raw >= saturation_limit) {
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::saturated);
        }
        if (ranges_m[index] < calibration.minimum_valid_range_m) {
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::below_minimum_range);
        }
        if (overlap < calibration.minimum_retrieval_overlap) {
            // overlap 太低时除法会把标定误差和噪声一起无限放大，因此完全禁止反演。
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::overlap_unusable);
        } else if (overlap < calibration.minimum_quantitative_overlap) {
            // 部分重叠区可保留定性光学结构，但不允许进入 PM/热点等定量产品。
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::partial_overlap);
        }

        bool detector_linear = std::isfinite(raw) && raw >= 0.0;
        /**
         * 当前仿真器使用非麻痹型响应 m = n / (1 + a*n)。解析反函数为
         * n = m / (1 - a*m)，其中 occupancy=a*m。occupancy 趋近 1 时反函数发散，
         * 所以先用 maximum_dead_time_occupancy 设置比数学奇点更保守的工程上限。
         */
        auto invert_dead_time = [&](double observed) {
            if (dead_time_loss <= 0.0) return observed;
            const double occupancy = dead_time_loss * observed;
            if (!std::isfinite(occupancy) || occupancy >= maximum_occupancy
                || occupancy >= 1.0) {
                detector_linear = false;
                return observed;
            }
            return observed / (1.0 - occupancy);
        };
        // 总计数和背景都经过同一个探测器，必须在相同响应域分别线性化后再相减。
        const double corrected_total = invert_dead_time(std::max(raw, 0.0));
        const double corrected_background = invert_dead_time(observed_background);
        if (!detector_linear) {
            quality |= lidar_core::quality_mask(
                lidar_core::BinQualityFlag::dead_time_uncorrectable);
        }

        // afterpulse_kernel[k-1] 表示前 k 个距离门泄漏到当前门的比例。
        double afterpulse = 0.0;
        double background_afterpulse = 0.0;
        for (std::size_t lag = 1;
             lag <= calibration.afterpulse_kernel.size() && lag <= index; ++lag) {
            const double coefficient = calibration.afterpulse_kernel[lag - 1];
            if (!std::isfinite(coefficient) || coefficient <= 0.0) continue;
            // 对总计数和背景应用同一卷积核，避免把稳态背景的拖尾误认为大气信号。
            afterpulse += coefficient * primary_total[index - lag];
            background_afterpulse += coefficient * primary_background[index - lag];
            if (!reconstruction_valid[index - lag]) {
                // 前序样本未知意味着当前 afterpulse 扣除量也未知，不能把递推误差隐藏掉。
                quality |= lidar_core::quality_mask(
                    lidar_core::BinQualityFlag::upstream_invalid);
            }
        }
        // 计数不允许为负；负值通常意味着背景/afterpulse 统计涨落，按零信号处理。
        primary_total[index] = std::max(corrected_total - afterpulse, 0.0);
        primary_background[index] = std::max(
            corrected_background - background_afterpulse, 0.0);
        reconstruction_valid[index] = detector_linear
            && !lidar_core::has_quality(quality, lidar_core::BinQualityFlag::saturated)
            && !lidar_core::has_quality(quality, lidar_core::BinQualityFlag::non_finite_input);

        // 至此才得到背景扣除后的净大气回波信号。
        const double signal = std::max(
            primary_total[index] - primary_background[index], 0.0);

        // SNR 使用观测域计数的 Poisson 近似：S*sqrt(N)/sqrt(S+B+B)。
        // 这里分母写 raw+background，与 raw=(signal+background) 等价于 signal+2*background。
        const double observed_signal = std::max(raw - observed_background, 0.0);
        output.snr[index] = observed_signal * pulse_gain
            / std::max(std::sqrt(std::max(raw + observed_background, 0.0)), 1e-9);
        if (output.snr[index] < config.minimum_channel_snr) {
            // 低 SNR 直接阻断反演，防止远端噪声通过 r^2 和 overlap 校正被大幅放大。
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::low_snr);
        }

        // 只有通过所有反演级质量门的 bin 才执行增益和 overlap 除法。
        output.corrected[index] = lidar_core::bin_is_usable_for_retrieval(quality)
            ? signal / receiver_gain / overlap
            : invalid;
    }
    return output;
}

/**
 * @brief 校正并拼接近、远场平行偏振通道，生成统一的 L1/反演输入。
 *
 * 双望远镜“零盲区”的软件含义不是忽略 overlap，而是优先使用当前距离上质量更好的物理
 * 通道：近场望远镜负责远场通道几何重叠不足的区段，远场望远镜负责近场灵敏度下降后的
 * 区段；共同有效区用稳健比例把二者归一到同一幅度，再连续过渡。
 *
 * 返回的 attenuated_backscatter 是经过能量归一和 r^2 校正的反演代理量，不是经过绝对
 * 系统常数标定的厂家后向散射产品。每个数组都与 profile.ranges_m 严格对齐。
 */
lidar_core::PreprocessResult preprocess_receiver_channels(
    const lidar_core::LidarProfile& profile,
    int integrated_pulses,
    const std::string& signal_unit,
    const ProcessorConfig& config) {
    // 每脉冲平均计数的方差随 N 脉冲平均降低为 1/N，因此 SNR 恢复因子为 sqrt(N)。
    const double pulse_gain = std::sqrt(static_cast<double>(std::max(integrated_pulses, 1)));

    // integrated_counts 表示帧中存的是 N 个脉冲总和；先除以 N，后续标定参数统一按每脉冲计数解释。
    const double raw_scale = signal_unit == "integrated_counts"
        ? 1.0 / static_cast<double>(std::max(integrated_pulses, 1))
        : 1.0;
    const auto* near_parallel = find_channel(profile, "near", "parallel");
    const auto* far_parallel = find_channel(profile, "far", "parallel");
    if (near_parallel == nullptr || far_parallel == nullptr) {
        /*
         * 兼容单主通道设备：把 profile 外层拼接通道包装成普通 LidarChannel，复用完全相同
         * 的探测器、背景和质量控制逻辑。该路径无法重新验证厂家内部 gluing，所以必须
         * 添加 receiver-channel-fusion-unavailable QC。
         */
        lidar_core::LidarChannel main;
        main.channel_id = "stitched_parallel";
        main.telescope = "stitched";
        main.polarization = "parallel";
        main.relative_gain = 1.0;
        main.background_counts = profile.background_counts;
        main.raw_counts = profile.raw_counts;
        main.overlap = profile.overlap;
        main.device_bin_quality = profile.device_bin_quality;
        ChannelSignal corrected = correct_channel(
            main,
            profile.ranges_m,
            raw_scale,
            pulse_gain,
            config.receiver_calibration,
            config);
        lidar_core::PreprocessResult output;
        // L1 净信号、SNR 和逐 bin QC 直接来自单通道校正结果。
        output.l1_signal = corrected.corrected;
        output.snr = corrected.snr;
        output.bin_quality = corrected.quality;
        output.attenuated_backscatter.resize(profile.ranges_m.size(),
            std::numeric_limits<double>::quiet_NaN());
        // 能量缺失时仍保留 L1 净信号，但禁止构造需要能量归一化的反演输入。
        const bool energy_valid = std::isfinite(profile.laser_energy_mj)
            && profile.laser_energy_mj > 1e-9;
        for (std::size_t index = 0; index < profile.ranges_m.size(); ++index) {
            if (!energy_valid) {
                output.bin_quality[index] |= lidar_core::quality_mask(
                    lidar_core::BinQualityFlag::invalid_laser_energy);
            }
            if (!lidar_core::bin_is_usable_for_retrieval(output.bin_quality[index])) continue;
            const double range_km = profile.ranges_m[index] / 1000.0;
            // P(r) 含固有 1/r^2 衰减，因此乘 r^2；再除单脉冲激光能量消除发射能量波动。
            output.attenuated_backscatter[index] = std::max(
                output.l1_signal[index] / profile.laser_energy_mj * range_km * range_km,
                1e-9);
        }
        output.qc_flags.emplace_back("receiver-channel-fusion-unavailable");
        if (!corrected.background_measured) {
            output.qc_flags.emplace_back("receiver-background-metadata-fallback");
        }
        return output;
    }

    // 双通道路径：近、远场必须先各自独立校正，不能先拼原始计数再除一条混合 overlap。
    ChannelSignal near = correct_channel(
        *near_parallel, profile.ranges_m, raw_scale, pulse_gain,
        config.receiver_calibration, config);
    ChannelSignal far = correct_channel(
        *far_parallel, profile.ranges_m, raw_scale, pulse_gain,
        config.receiver_calibration, config);

    /*
     * 在配置的 gluing 共同区估算 near/far 幅度比例。只有两路都通过反演级 QC 的 bin 才
     * 能作为比例样本，否则低 SNR 或低 overlap 会把尺度因子带偏。使用中位数进一步抑制
     * 局地云和污染羽流造成的尖峰。
     */
    std::vector<double> ratios;
    for (std::size_t index = 0; index < profile.ranges_m.size(); ++index) {
        const double range = profile.ranges_m[index];
        if (range >= config.glue_start_range_m && range <= config.glue_stop_range_m
            && lidar_core::bin_is_usable_for_retrieval(near.quality[index])
            && lidar_core::bin_is_usable_for_retrieval(far.quality[index])
            && far.corrected[index] > 1e-12) {
            ratios.push_back(near.corrected[index] / far.corrected[index]);
        }
    }
    // 少于 5 个共同有效点时证据不足，退化为 1.0 并显式报告 QC，而不是静默猜测。
    const bool gluing_ratio_verified = ratios.size() >= 5;
    const double far_scale = gluing_ratio_verified
        ? std::clamp(median(std::move(ratios)), 0.05, 20.0)
        : 1.0;

    lidar_core::PreprocessResult output;
    // 所有产品数组预分配到完整距离轴长度；无效数值稍后写为 NaN，质量原因写入 bin_quality。
    output.l1_signal.resize(profile.ranges_m.size(), 0.0);
    output.attenuated_backscatter.resize(profile.ranges_m.size(), 1e-9);
    output.snr.resize(profile.ranges_m.size(), 0.0);
    output.bin_quality.resize(profile.ranges_m.size(), 0);
    const double glue_span = std::max(config.glue_stop_range_m - config.glue_start_range_m, 1.0);
    const bool energy_valid = std::isfinite(profile.laser_energy_mj)
        && profile.laser_energy_mj > 1e-9;
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    int low_snr_bins = 0;
    int masked_near_bins = 0;
    int partial_overlap_bins = 0;
    int saturated_bins = 0;
    int dead_time_bins = 0;
    for (std::size_t index = 0; index < profile.ranges_m.size(); ++index) {
        // 先把距离线性映射到 [0,1]，0 代表完全使用近场，1 代表完全使用远场。
        double far_weight = std::clamp(
            (profile.ranges_m[index] - config.glue_start_range_m) / glue_span,
            0.0,
            1.0);
        // smoothstep 的一阶导数在两端为 0，可避免线性拼接在 75/300 m 产生斜率折点。
        far_weight = far_weight * far_weight * (3.0 - 2.0 * far_weight);

        // retrieval usable 允许部分 overlap；quantitative usable 还要求达到定量 overlap 阈值。
        const bool near_usable = lidar_core::bin_is_usable_for_retrieval(near.quality[index]);
        const bool far_usable = lidar_core::bin_is_usable_for_retrieval(far.quality[index]);
        const bool near_quantitative = near_usable
            && lidar_core::bin_is_usable_for_quantitative_product(near.quality[index]);
        const bool far_quantitative = far_usable
            && lidar_core::bin_is_usable_for_quantitative_product(far.quality[index]);
        if (!far_usable && near_usable) {
            // 远场无效时无条件使用近场，不让无效通道通过权重混入结果。
            far_weight = 0.0;
        } else if (!near_usable && far_usable) {
            far_weight = 1.0;
        } else if (near_quantitative && !far_quantitative) {
            // 两路都能做光学反演时，仍优先选择达到定量 overlap 的一路，尽量扩大可靠 PM 区。
            far_weight = 0.0;
        } else if (!near_quantitative && far_quantitative) {
            far_weight = 1.0;
        }

        auto& quality = output.bin_quality[index];
        double corrected = invalid;
        if (near_usable && far_usable) {
            // 共同有效区把 far 乘以稳健尺度后加权；中间区质量位取两路并集。
            corrected = (1.0 - far_weight) * near.corrected[index]
                + far_weight * far.corrected[index] * far_scale;
            quality = far_weight <= 0.0 ? near.quality[index]
                : (far_weight >= 1.0 ? far.quality[index]
                                     : near.quality[index] | far.quality[index]);
        } else if (near_usable) {
            // 单路有效时只传播实际被选择通道的质量位，另一通道故障不应误伤输出。
            corrected = near.corrected[index];
            quality = near.quality[index];
        } else if (far_usable) {
            corrected = far.corrected[index] * far_scale;
            quality = far.quality[index];
        } else {
            // 两路都不可用时保留所有原始原因，并追加 no_valid_channel 便于统计和 UI 解释。
            quality = near.quality[index] | far.quality[index]
                | lidar_core::quality_mask(lidar_core::BinQualityFlag::no_valid_channel);
        }
        if (!energy_valid) {
            quality |= lidar_core::quality_mask(lidar_core::BinQualityFlag::invalid_laser_energy);
        }
        output.l1_signal[index] = corrected;
        if (lidar_core::bin_is_usable_for_retrieval(quality)) {
            const double range_km = profile.ranges_m[index] / 1000.0;
            // 这里输出的是能量和距离校正后的相对量；1e-9 只保护 Fernald 的对数/除法稳定性。
            output.attenuated_backscatter[index] = std::max(
                corrected / profile.laser_energy_mj * range_km * range_km, 1e-9);
        } else {
            output.attenuated_backscatter[index] = invalid;
        }
        // SNR 使用与信号相同的通道权重；两路都无效时写 0，并由质量位说明无效原因。
        output.snr[index] = near_usable && far_usable
            ? (1.0 - far_weight) * near.snr[index] + far_weight * far.snr[index]
            : (near_usable ? near.snr[index] : (far_usable ? far.snr[index] : 0.0));
        // 同时累计廓线级摘要，避免 UI 必须逐 bin 解码全部位掩码才能显示主要问题。
        low_snr_bins += output.snr[index] < config.minimum_channel_snr ? 1 : 0;
        masked_near_bins += lidar_core::has_quality(
            quality, lidar_core::BinQualityFlag::below_minimum_range) ? 1 : 0;
        partial_overlap_bins += lidar_core::has_quality(
            quality, lidar_core::BinQualityFlag::partial_overlap) ? 1 : 0;
        saturated_bins += lidar_core::has_quality(
            quality, lidar_core::BinQualityFlag::saturated) ? 1 : 0;
        dead_time_bins += lidar_core::has_quality(
            quality, lidar_core::BinQualityFlag::dead_time_uncorrectable) ? 1 : 0;
    }
    // 以下标志描述整条射线采用的处理路径或主要质量问题；具体位置仍以 bin_quality 为准。
    output.qc_flags.emplace_back("near-far-channels-glued");
    if (!gluing_ratio_verified) {
        output.qc_flags.emplace_back("channel-gluing-ratio-unverified");
    }
    if (!near.background_measured || !far.background_measured) {
        output.qc_flags.emplace_back("receiver-background-metadata-fallback");
    }
    if (near.dead_time_corrected || far.dead_time_corrected) {
        output.qc_flags.emplace_back("dead-time-correction-applied");
    }
    if (near.afterpulse_corrected || far.afterpulse_corrected) {
        output.qc_flags.emplace_back("afterpulse-correction-applied");
    }
    if (low_snr_bins * 2 > static_cast<int>(profile.ranges_m.size())) {
        output.qc_flags.emplace_back("low-snr-majority-of-range");
    }
    if (masked_near_bins > 0) {
        output.qc_flags.emplace_back("near-range-bins-masked");
    }
    if (partial_overlap_bins > 0) {
        output.qc_flags.emplace_back("partial-overlap-excluded-from-quantitative-products");
    }
    if (saturated_bins > 0) {
        output.qc_flags.emplace_back("saturated-bins-masked");
    }
    if (dead_time_bins > 0) {
        output.qc_flags.emplace_back("dead-time-uncorrectable-bins-masked");
    }
    return output;
}

/** @brief 向周期 QC 追加字符串并保持首次出现顺序，避免同一问题被每条射线重复展示。 */
void append_unique(std::vector<std::string>& output, const std::vector<std::string>& values) {
    for (const auto& value : values) {
        if (std::find(output.begin(), output.end(), value) == output.end()) {
            output.push_back(value);
        }
    }
}

} // namespace

FrameProcessor::FrameProcessor(ProcessorConfig config)
    : config_(std::move(config)) {
}

/** @brief 注册周期交付回调；处理器本身不跨线程投递，调用线程负责线程亲和性。 */
void FrameProcessor::set_step_complete_callback(StepCompleteCallback callback) {
    on_step_complete_ = std::move(callback);
}

void FrameProcessor::set_pm_calibration(const PmCalibrationModel& calibration) {
    // 标定切换必须发生在周期边界，避免同一 PPI 圈的一部分射线使用旧系数、另一部分用新系数。
    finalize_step();
    config_.pm_calibration = calibration;
}

void FrameProcessor::set_receiver_calibration(
    const ReceiverCalibrationModel& calibration) {
    // 接收机标定会改变距离轴、有效区和 L1 幅度，因此同样先封口当前周期。
    finalize_step();
    config_.receiver_calibration = calibration;
}

/**
 * @brief 把一条统一设备廓线从 L0 处理到质量受控 L2。
 *
 * 该函数只处理一条射线，不负责扫描周期完整性。输入 profile 已由协议层转换为公共模型；
 * frame.payload 仍提供积分脉冲、计数单位和探测模式等不属于 LidarProfile 的采集元数据。
 * 返回对象保留原始廓线、所有中间产品、逐 bin QC、廓线 QC 和处理耗时，方便科学复核。
 */
lidar_core::ProcessedProfile FrameProcessor::process_device_profile(
    const lidar_protocol::Frame& frame,
    lidar_core::LidarProfile profile,
    std::vector<std::string>& qc_flags) const {
    const auto started = std::chrono::steady_clock::now();

    // ---- 1. 距离轴与兼容字段准备 -------------------------------------------------
    // 电缆/触发延迟会让协议距离轴整体偏移。先校正距离，后续背景区、overlap 和 ENU 都使用同一轴。
    if (std::abs(config_.receiver_calibration.range_zero_offset_m) > 1e-12) {
        for (double& range_m : profile.ranges_m) {
            range_m -= config_.receiver_calibration.range_zero_offset_m;
        }
        qc_flags.emplace_back("range-zero-correction-applied");
    }
    if (profile.overlap.empty() && !profile.ranges_m.empty()) {
        /*
         * 兼容只提供主计数、不提供主 overlap 的适配器。数值上填 1.0 仅为保证结构完整；
         * 同时把所有主通道 bin 标为 partial_overlap，确保它们不能被误用为定量 PM。
         * 四物理通道齐全时实际预处理使用各自 overlap，不使用这个兼容主通道假设。
         */
        profile.overlap.assign(profile.ranges_m.size(), 1.0);
        if (profile.device_bin_quality.empty()) {
            profile.device_bin_quality.assign(
                profile.ranges_m.size(),
                lidar_core::quality_mask(lidar_core::BinQualityFlag::partial_overlap));
        }
        qc_flags.emplace_back("main-channel-overlap-assumed-unity");
    }
    // 校验必须在任何按索引访问之前完成，防止错误帧造成越界或非有限值扩散。
    validate_profile(profile);

    // ---- 2. 采集单位、波长和探测模式合同 -----------------------------------------
    // integrated_pulses 至少为 1；缺失时退化为单脉冲，SNR 不获得虚假的积分增益。
    const int integrated_pulses = std::max(
        1,
        static_cast<int>(std::llround(json_number(frame.payload, "integrated_pulses", 1.0))));
    // 532 nm 是当前 YLJ5 公开规格和分子参考回退的中心波长。
    const double wavelength_nm = json_number(frame.payload, "wavelength_nm", 532.0);
    std::string signal_unit = json_string(frame.payload, "signal_unit");
    if (signal_unit.empty()) {
        // 帧未声明单位时使用接收机标定合同，但必须留下 QC 供数据追溯。
        signal_unit = config_.receiver_calibration.signal_unit;
        qc_flags.emplace_back("signal-unit-from-receiver-calibration");
    }
    if (signal_unit != "mean_counts_per_pulse" && signal_unit != "integrated_counts") {
        // 未知单位无法安全套用死时间/饱和系数；未实测标定模式下保守采用仿真协议默认值。
        signal_unit = "mean_counts_per_pulse";
        qc_flags.emplace_back("signal-unit-unknown-assumed-mean-counts-per-pulse");
    }
    if (config_.receiver_calibration.valid
        && signal_unit != config_.receiver_calibration.signal_unit) {
        // 有效标定的系数带单位，错单位继续计算会产生数量级正确但物理错误的结果，直接拒绝。
        throw std::runtime_error("frame signal unit conflicts with receiver calibration");
    }
    std::string detector_mode = json_string(frame.payload, "detector_mode");
    if (detector_mode == "simulated_photon_counting") {
        // 仿真器的来源标签归一化为实际算法关心的 photon_counting 模式。
        detector_mode = "photon_counting";
    }
    if (detector_mode.empty()) {
        // 与计数单位相同，帧缺失时由接收机标定合同补齐，并显式报告来源。
        detector_mode = config_.receiver_calibration.detector_mode;
        qc_flags.emplace_back("detector-mode-from-receiver-calibration");
    }
    if (detector_mode != "photon_counting" && detector_mode != "analog") {
        detector_mode = config_.receiver_calibration.detector_mode;
        qc_flags.emplace_back("detector-mode-unknown-used-calibration-mode");
    }
    if (config_.receiver_calibration.valid
        && detector_mode != config_.receiver_calibration.detector_mode) {
        // 模拟采集没有光子计数死时间；探测模式错配时绝不能静默应用另一套响应公式。
        throw std::runtime_error("frame detector mode conflicts with receiver calibration");
    }
    if (!config_.receiver_calibration.valid) {
        // 默认参数只用于公开规格数字孪生；该标志阻止外部把结果宣称为实机标定产品。
        qc_flags.emplace_back("receiver-calibration-assumed-not-device-verified");
    }

    // ---- 3. 分子参考 -------------------------------------------------------------
    // 实时仿真协议默认不发送真值分子场，缺失时用标准大气回退为 Fernald 提供边界条件。
    fill_standard_molecular_reference(
        profile,
        site_info_.altitude_m,
        wavelength_nm,
        qc_flags);

    // ---- 4. 分通道 L1 校正与近远场拼接 -------------------------------------------
    // 对本条射线复制有效配置，允许根据探测模式关闭不适用的校正而不修改处理器全局状态。
    ProcessorConfig effective_config = config_;
    if (detector_mode == "analog") {
        // 模拟 ADC 采集不存在光子事件计数器恢复时间，dead-time 解析逆在此模式不适用。
        effective_config.receiver_calibration.dead_time_loss_per_count = 0.0;
        qc_flags.emplace_back("analog-detector-dead-time-correction-not-applicable");
    }
    lidar_core::PreprocessResult preprocessed = preprocess_receiver_channels(
        profile,
        integrated_pulses,
        signal_unit,
        effective_config);
    append_unique(qc_flags, preprocessed.qc_flags);

    // ---- 5. Fernald/Klett 光学反演 ------------------------------------------------
    // 反演函数读取逐 bin QC，只在最长连续有效距离段上积分；无效位置返回 NaN。
    lidar_core::FernaldInversionStep inversion(config_.retrieval);
    auto optical = inversion.process(
        profile, preprocessed.attenuated_backscatter, preprocessed.bin_quality);
    int retrieval_unavailable_bins = 0;
    for (std::size_t index = 0; index < optical.first.size(); ++index) {
        if (!std::isfinite(optical.first[index])) {
            // 预处理可用但未落入最终连续反演区的 bin，也要追加 retrieval_unavailable 原因。
            preprocessed.bin_quality[index] |= lidar_core::quality_mask(
                lidar_core::BinQualityFlag::retrieval_unavailable);
            ++retrieval_unavailable_bins;
        }
    }
    if (retrieval_unavailable_bins == static_cast<int>(optical.first.size())) {
        // 整条射线仍保留用于故障分析，但不能生成任何 L2 定量产品。
        qc_flags.emplace_back("retrieval-insufficient-contiguous-valid-range");
    } else if (retrieval_unavailable_bins > 0) {
        qc_flags.emplace_back("retrieval-invalid-bins-masked");
    }
    // 湿度增长会抬高环境消光；除以增长因子得到与干态 PM 标定一致的消光。
    lidar_core::HumidityCorrectionStep humidity(config_.humidity);
    std::vector<double> dry_extinction = humidity.process(optical.first, profile.relative_humidity);

    // ---- 6. PM 标定门控 -----------------------------------------------------------
    // 未具备完整组合标定时保持空数组；具备标定但单 bin 无效时保留等长数组并写 NaN。
    std::vector<double> pm25;
    std::vector<double> pm10;
    if (config_.pm_calibration.valid && config_.receiver_calibration.valid) {
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        pm25.resize(dry_extinction.size(), invalid);
        pm10.resize(dry_extinction.size(), invalid);
        int quantitative_masked_bins = 0;
        for (std::size_t index = 0; index < dry_extinction.size(); ++index) {
            // 定量产品除反演有效外，还要求不处于 partial_overlap 区。
            if (!std::isfinite(dry_extinction[index])
                || !lidar_core::bin_is_usable_for_quantitative_product(
                    preprocessed.bin_quality[index])) {
                ++quantitative_masked_bins;
                continue;
            }
            // 当前模型是站点共址标定的线性映射；负预测没有物理意义，截断到 0 ug/m^3。
            pm25[index] = std::max(
                0.0,
                config_.pm_calibration.pm25_intercept_ugm3
                    + config_.pm_calibration.pm25_slope_ugm3_per_km
                        * dry_extinction[index]);
            pm10[index] = std::max(
                0.0,
                config_.pm_calibration.pm10_intercept_ugm3
                    + config_.pm_calibration.pm10_slope_ugm3_per_km
                        * dry_extinction[index]);
        }
        if (quantitative_masked_bins > 0) {
            qc_flags.emplace_back("quantitative-product-invalid-bins-masked");
        }
    } else if (config_.pm_calibration.valid) {
        // PM 回归系数存在但接收机幅度/有效区未标定时，仍不允许宣称定量 PM。
        qc_flags.emplace_back("receiver-calibration-required-for-pm");
    } else {
        qc_flags.emplace_back("pm-calibration-missing");
    }

    // ---- 7. 几何投影与结果封装 ----------------------------------------------------
    // ENU 与科学产品数组保持等长；无效 bin 仍保留坐标，便于 UI 显示其空间缺口。
    lidar_core::CoordinateProjectionStep projection;
    auto enu_points = projection.process(profile);
    const auto ended = std::chrono::steady_clock::now();
    // 聚合初始化顺序与 ProcessedProfile 字段定义一致，所有大数组通过 move 避免 5334-bin 复制。
    return lidar_core::ProcessedProfile{
        std::move(profile),
        std::move(preprocessed.l1_signal),
        std::move(preprocessed.attenuated_backscatter),
        std::move(preprocessed.snr),
        std::move(optical.first),
        std::move(dry_extinction),
        std::move(pm25),
        std::move(pm10),
        std::move(enu_points),
        std::move(preprocessed.bin_quality),
        qc_flags,
        std::chrono::duration<double, std::milli>(ended - started).count(),
    };
}

/**
 * @brief 按协议到达顺序消费一帧，并维护当前扫描周期的聚合状态。
 *
 * 网络线程已经保证 JSONL 帧顺序，本函数不排序射线。lidar_raw 逐条执行完整算法；状态帧
 * 只更新随后射线使用的站点元数据；heartbeat 负责把当前周期封口。单条坏射线不会终止
 * 整个周期，而是增加 rejected_count 并给周期添加统一错误 QC。
 */
void FrameProcessor::handle_frame(const lidar_protocol::Frame& frame) {
    switch (frame.type) {
    case lidar_protocol::FrameType::lidar_raw: {
        // 调试日志仅打印截断后的 JSON 预览；frame 本身保持原样继续进入协议映射和算法。
        LIDAR_LOG_INFO("[client] lidar_raw frame preview:\n", make_debug_frame_json(frame));
        // 某些真实协议没有显式 heartbeat，时间戳变化也作为确定的周期边界。
        if (!current_timestamp_.empty() && frame.timestamp != current_timestamp_) {
            finalize_step();
        }
        current_timestamp_ = frame.timestamp;
        // 扫描模式属于周期元数据；非空值才覆盖，避免缺字段帧清掉已经识别的模式。
        const std::string scan_pattern = json_string(frame.payload, "azimuth_scan_pattern");
        if (!scan_pattern.empty()) {
            current_scan_pattern_ = scan_pattern;
        }
        ++current_raw_count_;
        try {
            // 协议层只负责字段映射；所有数组和物理边界校验都在 process_device_profile 内完成。
            lidar_core::LidarProfile profile = lidar_protocol::json_to_profile(frame.payload);
            profile.timestamp = frame.timestamp;
            std::vector<std::string> profile_qc;
            auto processed = process_device_profile(frame, std::move(profile), profile_qc);
            // 周期级 QC 去重，但每条 ProcessedProfile 内仍保留其完整 QC 副本。
            append_unique(current_qc_flags_, profile_qc);
            current_processed_.push_back(std::move(processed));
        } catch (const std::exception&) {
            // 当前协议未公开向 UI 透传异常文本，避免把实现细节或不可信输入带到界面。
            ++current_rejected_count_;
            append_unique(current_qc_flags_, {"malformed-or-unprocessable-lidar-frame"});
        }
        break;
    }
    case lidar_protocol::FrameType::ground_obs: {
        // 地面 PM/气象观测不参与单射线反演，只与同周期结果一起交付供标定和对照。
        auto ground = lidar_protocol::json_to_ground(frame.payload);
        if (ground.timestamp.empty()) {
            ground.timestamp = frame.timestamp;
        }
        current_ground_.push_back(std::move(ground));
        break;
    }
    case lidar_protocol::FrameType::status:
    case lidar_protocol::FrameType::telemetry:
        // 标准大气回退至少需要站点海拔；状态与遥测均采用增量更新语义。
        if (frame.payload.contains("site_id") && frame.payload.at("site_id").is_string()) {
            site_info_.site_id = frame.payload.at("site_id").string_value();
        }
        if (frame.payload.contains("site_name") && frame.payload.at("site_name").is_string()) {
            site_info_.name = frame.payload.at("site_name").string_value();
        }
        if (frame.payload.contains("altitude_m") && frame.payload.at("altitude_m").is_number()) {
            site_info_.altitude_m = frame.payload.at("altitude_m").number_value();
        }
        break;
    case lidar_protocol::FrameType::heartbeat:
        // heartbeat 表示设备已完成本周期所有射线和辅助帧，可以安全交付不可变快照。
        finalize_step();
        break;
    default:
        break;
    }
}

/**
 * @brief 汇总并交付当前扫描周期，然后清空可变聚合状态。
 *
 * 封口阶段不再修改单射线科学产品，只计算周期统计、按仰角分组 PPI 射线，并在组合标定
 * 有效时执行热点检测。所有结果先移动到局部 StepResult，再清空成员，保证回调即使立即
 * 触发下一轮处理也不会看到上一个周期的残留状态。
 */
void FrameProcessor::finalize_step() {
    if (current_timestamp_.empty()) {
        // 没有接收过 lidar_raw 时不存在可交付周期，重复 heartbeat 是幂等的。
        return;
    }

    StepResult result;
    // 大数组使用 move 转移所有权；周期交付后处理器不再访问这些旧数据。
    result.timestamp = current_timestamp_;
    result.scan_pattern = current_scan_pattern_;
    result.processed_profiles = std::move(current_processed_);
    result.ground_measurements = std::move(current_ground_);
    result.qc_flags = std::move(current_qc_flags_);
    result.raw_count = current_raw_count_;
    result.rejected_count = current_rejected_count_;
    result.pm_calibrated = config_.pm_calibration.valid
        && config_.receiver_calibration.valid;
    result.calibration_id = config_.pm_calibration.calibration_id;
    result.receiver_calibrated = config_.receiver_calibration.valid;
    result.receiver_calibration_id = config_.receiver_calibration.calibration_id;

    // 热点算法只消费 PPI 射线；垂直 stare 单独计数并用于垂直廓线视图。
    std::vector<lidar_core::ProcessedProfile> ppi_profiles;
    std::set<int> elevations;
    double latency_sum = 0.0;
    for (const auto& processed : result.processed_profiles) {
        latency_sum += processed.latency_ms;
        // 周期总有效/屏蔽 bin 数采用反演级门控；partial_overlap 仍可计为光学有效。
        for (const auto quality : processed.bin_quality) {
            if (lidar_core::bin_is_usable_for_retrieval(quality)) {
                ++result.valid_bin_count;
            } else {
                ++result.masked_bin_count;
            }
        }
        if (processed.profile.scan_mode == "ppi") {
            ppi_profiles.push_back(processed);
            // 以 0.01 度量化仰角作为 map key，避免浮点微小扰动把同一层拆成多层。
            elevations.insert(static_cast<int>(std::llround(processed.profile.elevation_deg * 100.0)));
            ++result.ppi_count;
        } else if (processed.profile.scan_mode == "stare") {
            ++result.vertical_count;
        }
    }
    result.elevation_layer_count = static_cast<int>(elevations.size());
    result.mean_processing_latency_ms = result.processed_profiles.empty()
        ? 0.0
        : latency_sum / static_cast<double>(result.processed_profiles.size());

    if (result.pm_calibrated && !ppi_profiles.empty()) {
        // detect_hotspots_from_processed 内部还会再次检查逐 bin 定量质量位和有限值。
        result.hotspots = lidar_core::detect_hotspots_from_processed(ppi_profiles, config_.hotspot);
    }

    // 回调前先把处理器恢复为空周期，避免回调重入时混入旧计数。
    current_timestamp_.clear();
    current_scan_pattern_.clear();
    current_processed_.clear();
    current_ground_.clear();
    current_qc_flags_.clear();
    current_raw_count_ = 0;
    current_rejected_count_ = 0;
    if (on_step_complete_) {
        // StepResult 只交付一次，移动后本函数不再读取 result。
        on_step_complete_(std::move(result));
    }
}

} // namespace lidar_client
