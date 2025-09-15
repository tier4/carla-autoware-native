// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <array>
#include <cmath>

namespace carla {
namespace ros2 {
namespace autoware_steering_compensation {

// Helper alias to use template type deduction
using _p = std::pair<float, float>;

// Steering compensation lookup table
// Pairs of (actual_steering_angle, ratio) where ratio = desired_angle / actual_angle
// Data from steer-angle-experiments.ods Sheet2
// Only positive values stored; absolute value used for lookup
constexpr std::array STEERING_COMPENSATION_TABLE{
  _p{0.007f, 14.286f},  // 0.007 rad actual -> 14.286 ratio
  _p{0.027f, 7.407f},   // 0.027 rad actual -> 7.407 ratio
  _p{0.061f, 4.918f},   // 0.061 rad actual -> 4.918 ratio
  _p{0.1085f, 3.687f},  // 0.1085 rad actual -> 3.687 ratio
  _p{0.170f, 2.941f},   // 0.170 rad actual -> 2.941 ratio
  _p{0.2445f, 2.454f},  // 0.2445 rad actual -> 2.454 ratio
  _p{0.3345f, 2.093f},  // 0.3345 rad actual -> 2.093 ratio
  _p{0.439f, 1.822f},   // 0.439 rad actual -> 1.822 ratio
  _p{0.560f, 1.607f},   // 0.560 rad actual -> 1.607 ratio
  _p{0.7005f, 1.428f},  // 0.7005 rad actual -> 1.428 ratio
  _p{0.8625f, 1.275f},  // 0.8625 rad actual -> 1.275 ratio
  _p{1.0565f, 1.136f}   // 1.0565 rad actual -> 1.136 ratio
};

// Linear interpolation function
inline float Lerp(const float a, const float b, const float t) {
  return a + t * (b - a);
}

// Get steering compensation ratio from lookup table with LERP
inline float GetSteeringCompensationRatio(const float actual_steering_angle) {
  // Use absolute value for symmetric steering
  const float abs_angle = std::abs(actual_steering_angle);

  // Handle edge cases
  if (abs_angle <= STEERING_COMPENSATION_TABLE.front().first) {
    return STEERING_COMPENSATION_TABLE.front().second;
  }
  if (abs_angle >= STEERING_COMPENSATION_TABLE.back().first) {
    return STEERING_COMPENSATION_TABLE.back().second;
  }

  // Find the two points to interpolate between
  for (size_t i = 0; i < STEERING_COMPENSATION_TABLE.size() - 1; ++i) {
    const auto [current_angle, current_compensation_ratio] = STEERING_COMPENSATION_TABLE[i];
    const auto [next_angle, next_compensation_ratio] = STEERING_COMPENSATION_TABLE[i + 1];

    if (abs_angle >= current_angle && abs_angle <= next_angle) {
      // Calculate interpolation factor
      const float t = (abs_angle - current_angle) / (next_angle - current_angle);
      // Interpolate between the two ratio values
      return Lerp(current_compensation_ratio, next_compensation_ratio, t);
    }
  }

  // Default fallback (should not reach here)
  return 1.0f;
}

} // namespace autoware_steering_compensation
} // namespace ros2
} // namespace carla
