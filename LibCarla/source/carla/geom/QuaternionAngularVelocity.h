// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <array>
#include <cmath>

namespace carla {
namespace geom {

  /// Average world-frame angular velocity (x, y, z), in radians per second.
  /// Inputs are nonzero rotation quaternions in (w, x, y, z) order. Computes
  /// the Hamilton product current * conjugate(previous), equivalent to UE's
  /// current * previous.Inverse() for unit quaternions (B is applied before A
  /// in A * B). No frame or handedness conversion is performed.
  ///
  /// The shortest-path quaternion log map uses atan2(|xyz|, w), so uniform
  /// quaternion scale, including small norm errors, does not affect the angle.
  /// Component quantization still limits the accuracy of the input rotations.
  /// Returns exactly zero for zero rotation or nonpositive dt (seconds).
  inline std::array<double, 3> AngularVelocityFromQuaternions(
      const std::array<double, 4> &previous,
      const std::array<double, 4> &current,
      double dt) {
    if (!(dt > 0.0)) {
      return {0.0, 0.0, 0.0};
    }
    double w = current[0] * previous[0] + current[1] * previous[1]
        + current[2] * previous[2] + current[3] * previous[3];
    double x = (current[1] * previous[0] - current[0] * previous[1])
        + (current[3] * previous[2] - current[2] * previous[3]);
    double y = (current[2] * previous[0] - current[0] * previous[2])
        + (current[1] * previous[3] - current[3] * previous[1]);
    double z = (current[3] * previous[0] - current[0] * previous[3])
        + (current[2] * previous[1] - current[1] * previous[2]);

    if (w < 0.0) {
      w = -w;
      x = -x;
      y = -y;
      z = -z;
    }
    const double s = std::hypot(x, y, z);
    if (!(s > 0.0)) {
      return {0.0, 0.0, 0.0};
    }
    const double angle = 2.0 * std::atan2(s, w);
    const double scale = angle / (s * dt);
    return {x * scale, y * scale, z * scale};
  }

} // namespace geom
} // namespace carla
