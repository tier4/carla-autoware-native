// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>
#include <fastdds/dds/topic/TypeSupport.hpp>

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;

class FastDDSTypeRegistry {
public:
    static efd::TypeSupport Create(const std::string& type_name);
};

} // namespace ros2
} // namespace carla
