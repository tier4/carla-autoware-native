#pragma once

#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include "CameraInfo.h"

namespace carla {
namespace ros2 {
namespace cyclone_helpers {

inline void InitCameraInfo(sensor_msgs_msg_CameraInfo& info,
                           uint32_t height, uint32_t width, double fov,
                           std::vector<double>& d_backing,
                           std::string& distortion_model_backing) {
    info.height = height;
    info.width = width;

    distortion_model_backing = "plumb_bob";
    info.distortion_model = const_cast<char*>(distortion_model_backing.c_str());

    const double cx = static_cast<double>(width) / 2.0;
    const double cy = static_cast<double>(height) / 2.0;
    const double fx = static_cast<double>(width) / (2.0 * std::tan(fov * M_PI / 360.0));
    const double fy = fx;

    d_backing = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.d._buffer = d_backing.data();
    info.d._length = 5;
    info.d._maximum = 5;
    info.d._release = false;

    info.k[0] = fx;  info.k[1] = 0.0; info.k[2] = cx;
    info.k[3] = 0.0; info.k[4] = fy;  info.k[5] = cy;
    info.k[6] = 0.0; info.k[7] = 0.0; info.k[8] = 1.0;

    info.r[0] = 1.0; info.r[1] = 0.0; info.r[2] = 0.0;
    info.r[3] = 0.0; info.r[4] = 1.0; info.r[5] = 0.0;
    info.r[6] = 0.0; info.r[7] = 0.0; info.r[8] = 1.0;

    info.p[0] = fx;  info.p[1] = 0.0; info.p[2] = cx;  info.p[3] = 0.0;
    info.p[4] = 0.0; info.p[5] = fy;  info.p[6] = cy;  info.p[7] = 0.0;
    info.p[8] = 0.0; info.p[9] = 0.0; info.p[10] = 1.0; info.p[11] = 0.0;

    info.binning_x = 0;
    info.binning_y = 0;
}

}  // namespace cyclone_helpers
}  // namespace ros2
}  // namespace carla
