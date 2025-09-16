// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <array>
#include <cmath>

namespace carla {
namespace ros2 {
/**
 * @note This solution is not ideal and it should be temporary. Ideally a root cause should be found
 * and some other solution should be implemented based on the findings.
 * This solution is purely based on experiments. For better accuracy more data points can be added
 * to the lookup table.
 */
namespace autoware_steering_compensation {
namespace detail {

constexpr std::tuple<float, float, float, float> MakeDataPoint(const float desired, const float actual) {
  return std::make_tuple(desired, actual, desired / actual, actual / desired);
}

// Steering compensation lookup table
// Tuples of (desired_steering_angle, actual_steering_angle, ratio1, ratio2) where:
//   ratio1 = desired_angle / actual_angle
//   ratio2 = actual_angle / desired_angle
// Data from steer-angle-experiments.ods Sheet2
// Only positive values stored; absolute value used for lookup
constexpr std::array DATA{
  MakeDataPoint(0.1f, 0.007f ),
  MakeDataPoint(0.2f, 0.027f ),
  MakeDataPoint(0.3f, 0.061f ),
  MakeDataPoint(0.4f, 0.1085f),
  MakeDataPoint(0.5f, 0.17f  ),
  MakeDataPoint(0.6f, 0.2445f),
  MakeDataPoint(0.7f, 0.3345f),
  MakeDataPoint(0.8f, 0.439f ),
  MakeDataPoint(0.9f, 0.56f  ),
  MakeDataPoint(1.0f, 0.7005f),
  MakeDataPoint(1.1f, 0.8625f),
  MakeDataPoint(1.2f, 1.0565f),
};

// Linear interpolation function
inline float Lerp(const float a, const float b, const float t) {
  return a + t * (b - a);
}

// Get steering compensation ratio from lookup table with LERP
template<std::size_t key_idx, std::size_t value_idx>
float GetSteeringCompensationRatio(const float angle) {
  // Use absolute value for symmetric steering
  const float key_angle = std::abs(angle);

  // Handle edge cases
  if (key_angle <= std::get<key_idx>(DATA.front())) {
    return std::get<value_idx>(DATA.front());
  }
  if (key_angle >= std::get<key_idx>(DATA.back())) {
    return std::get<value_idx>(DATA.back());
  }

  // Find the two points to interpolate between
  for (size_t i = 0; i < DATA.size() - 1; ++i) {
    const auto current_key_angle   = std::get<key_idx  >(DATA[i]);
    const auto current_value_angle = std::get<value_idx>(DATA[i]);
    const auto next_key_angle      = std::get<key_idx  >(DATA[i + 1]);
    const auto next_value_angle    = std::get<value_idx>(DATA[i + 1]);

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
} // namespace detail

inline float GetDesiredSteeringCompensationRatio(const float angle) {
  return detail::GetSteeringCompensationRatio<0, 3>(angle);
}

inline float GetActualSteeringCompensationRatio(const float angle) {
  return detail::GetSteeringCompensationRatio<1, 2>(angle);
}
} // namespace autoware_steering_compensation
} // namespace ros2
} // namespace carla
