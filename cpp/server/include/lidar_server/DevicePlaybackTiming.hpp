#pragma once

#include "lidar_protocol/Frame.hpp"
#include "lidar_server/SimDevice.hpp"

namespace lidar_server {

double acquisition_seconds_for_frame(
    const SimDeviceConfig& config,
    const lidar_protocol::Frame& frame);

int playback_delay_ms_for_frame(
    const SimDeviceConfig& config,
    const lidar_protocol::Frame& frame);

} // namespace lidar_server
