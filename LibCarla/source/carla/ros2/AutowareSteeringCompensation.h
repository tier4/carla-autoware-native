// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <array>
#include <cmath>

namespace carla {
namespace ros2 {
namespace autoware_steering_compensation {

constexpr std::tuple<float, float, float, float> __data_point(const float desired, const float actual) {
  return std::make_tuple(desired, actual, desired / actual, actual / desired);
}

// Steering compensation lookup table
// Tuples of (desired_steering_angle, actual_steering_angle, ratio1, ratio2) where:
//   ratio1 = desired_angle / actual_angle
//   ratio2 = actual_angle / desired_angle
// Data from steer-angle-experiments.ods Sheet2
// Only positive values stored; absolute value used for lookup
constexpr std::array STEERING_COMPENSATION_TABLE{
  __data_point(0.1f, 0.007f ),
  __data_point(0.2f, 0.027f ),
  __data_point(0.3f, 0.061f ),
  __data_point(0.4f, 0.1085f),
  __data_point(0.5f, 0.17f  ),
  __data_point(0.6f, 0.2445f),
  __data_point(0.7f, 0.3345f),
  __data_point(0.8f, 0.439f ),
  __data_point(0.9f, 0.56f  ),
  __data_point(1.0f, 0.7005f),
  __data_point(1.1f, 0.8625f),
  __data_point(1.2f, 1.0565f),
};

// Linear interpolation function
inline float Lerp(const float a, const float b, const float t) {
  return a + t * (b - a);
}

// Get steering compensation ratio from lookup table with LERP
template<std::size_t key_idx, std::size_t value_idx>
float __lookup_table_impl(const float angle) {
  // Use absolute value for symmetric steering
  const float key_angle = std::abs(angle);

  // Handle edge cases
  if (key_angle <= std::get<key_idx>(STEERING_COMPENSATION_TABLE.front())) {
    return std::get<value_idx>(STEERING_COMPENSATION_TABLE.front());
  }
  if (key_angle >= std::get<key_idx>(STEERING_COMPENSATION_TABLE.back())) {
    return std::get<value_idx>(STEERING_COMPENSATION_TABLE.back());
  }

  // Find the two points to interpolate between
  for (size_t i = 0; i < STEERING_COMPENSATION_TABLE.size() - 1; ++i) {
    const auto current_key_angle   = std::get<key_idx  >(STEERING_COMPENSATION_TABLE[i]);
    const auto current_value_angle = std::get<value_idx>(STEERING_COMPENSATION_TABLE[i]);
    const auto next_key_angle      = std::get<key_idx  >(STEERING_COMPENSATION_TABLE[i + 1]);
    const auto next_value_angle    = std::get<value_idx>(STEERING_COMPENSATION_TABLE[i + 1]);

    if (current_key_angle <= key_angle && key_angle <= next_key_angle) {
      // Calculate interpolation factor
      const float t = (key_angle - current_key_angle) / (next_key_angle - current_key_angle);
      // Interpolate between the two ratio values
      return Lerp(current_value_angle, next_value_angle, t);
    }
  }

  // Default fallback (should not reach here)
  return 1.0f;
}

inline float GetDesiredSteeringCompensationRatio(const float angle) {
  return __lookup_table_impl<0, 3>(angle);
}

inline float GetActualSteeringCompensationRatio(const float angle) {
  return __lookup_table_impl<1, 2>(angle);
}

} // namespace autoware_steering_compensation
} // namespace ros2
} // namespace carla
