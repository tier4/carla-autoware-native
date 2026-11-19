// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/geom/QuaternionAngularVelocity.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>

namespace {

using Quaternion = std::array<double, 4>; // w, x, y, z
using carla::geom::AngularVelocityFromQuaternions;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.005;
constexpr Quaternion kIdentity{1.0, 0.0, 0.0, 0.0};

Quaternion FromEuler(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);
  return {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy};
}

Quaternion Quantize(Quaternion q) {
  for (auto &component : q) {
    component = static_cast<double>(static_cast<float>(component));
  }
  return q;
}

Quaternion Scale(Quaternion q, double scale) {
  for (auto &component : q) {
    component *= scale;
  }
  return q;
}

void ExpectZero(const std::array<double, 3> &omega) {
  for (const double component : omega) {
    EXPECT_EQ(component, 0.0);
  }
}

class QuantizedYawRate : public ::testing::TestWithParam<double> {};

} // namespace

TEST(QuaternionAngularVelocity, identity_is_exactly_zero) {
  ExpectZero(AngularVelocityFromQuaternions(kIdentity, kIdentity, kDt));
}

TEST(QuaternionAngularVelocity, stationary_nonunit_rotation_is_exactly_zero) {
  for (const double norm : {1.0 - 1e-7, 1.0, 1.0 + 1e-7}) {
    SCOPED_TRACE(norm);
    const auto q = Scale(FromEuler(0.03, -0.02, 1.234), norm);
    ExpectZero(AngularVelocityFromQuaternions(q, q, kDt));
    const auto quantized = Quantize(q);
    ExpectZero(AngularVelocityFromQuaternions(quantized, quantized, kDt));
  }
}

TEST_P(QuantizedYawRate, preserves_world_yaw_at_random_headings) {
  std::mt19937 generator(123456u);
  std::uniform_real_distribution<double> heading(-kPi, kPi);
  for (int sample = 0; sample < 256; ++sample) {
    const double yaw = heading(generator);
    for (const double rate : {GetParam(), -GetParam()}) {
      SCOPED_TRACE(::testing::Message() << "sample=" << sample
          << " heading=" << yaw << " rate=" << rate);
      // Fixed roll/pitch: changing yaw is a rotation about world Z, not body Z.
      const auto previous = Quantize(FromEuler(0.03, -0.02, yaw));
      const auto current = Quantize(FromEuler(0.03, -0.02, yaw + rate * kDt));
      const auto omega = AngularVelocityFromQuaternions(previous, current, kDt);
      ASSERT_LT(std::abs(omega[2] - rate), 1e-4);
      ASSERT_LT(std::abs(omega[0]), 1e-4);
      ASSERT_LT(std::abs(omega[1]), 1e-4);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    SmallAndOrdinaryRates, QuantizedYawRate,
    ::testing::Values(0.005, 0.02, 0.1, 1.0));

TEST(QuaternionAngularVelocity, quaternion_sign_does_not_change_shortest_path) {
  const auto previous = FromEuler(0.03, -0.02, 1.234);
  for (const double angle : {0.0001, 0.8, 3.0}) {
    const auto current = FromEuler(0.03, -0.02, 1.234 + angle);
    for (const double previous_sign : {-1.0, 1.0}) {
      for (const double current_sign : {-1.0, 1.0}) {
        const auto omega = AngularVelocityFromQuaternions(
            Scale(previous, previous_sign), Scale(current, current_sign), kDt);
        EXPECT_NEAR(omega[0], 0.0, 1e-10);
        EXPECT_NEAR(omega[1], 0.0, 1e-10);
        EXPECT_NEAR(omega[2], angle / kDt, 1e-10);
      }
    }
  }
}

TEST(QuaternionAngularVelocity, three_radians_about_arbitrary_axis) {
  const Quaternion current{
      std::cos(1.5), 0.6 * std::sin(1.5), 0.0, 0.8 * std::sin(1.5)};
  const auto omega = AngularVelocityFromQuaternions(kIdentity, current, kDt);
  EXPECT_NEAR(omega[0], 0.6 * 3.0 / kDt, 1e-10);
  EXPECT_EQ(omega[1], 0.0);
  EXPECT_NEAR(omega[2], 0.8 * 3.0 / kDt, 1e-10);
}

TEST(QuaternionAngularVelocity, rotation_beyond_pi_takes_shortest_path) {
  const Quaternion current{std::cos(1.75), 0.0, std::sin(1.75), 0.0};
  const auto omega = AngularVelocityFromQuaternions(kIdentity, current, kDt);
  EXPECT_EQ(omega[0], 0.0);
  EXPECT_NEAR(omega[1], (3.5 - 2.0 * kPi) / kDt, 1e-10);
  EXPECT_EQ(omega[2], 0.0);
}

TEST(QuaternionAngularVelocity, delta_is_in_world_frame) {
  const double half_sqrt = std::sqrt(0.5);
  // Roll by pi/2, then yaw by pi/2 in the world frame.
  const Quaternion previous{half_sqrt, half_sqrt, 0.0, 0.0};
  const Quaternion current{0.5, 0.5, 0.5, 0.5};
  const auto omega = AngularVelocityFromQuaternions(previous, current, kDt);
  EXPECT_EQ(omega[0], 0.0);
  EXPECT_EQ(omega[1], 0.0);
  EXPECT_NEAR(omega[2], (kPi / 2.0) / kDt, 1e-10);
}

TEST(QuaternionAngularVelocity, tiny_rotations_preserve_each_axis) {
  constexpr double rate = 1e-9;
  for (size_t axis = 0; axis < 3; ++axis) {
    Quaternion current{std::cos(rate * kDt / 2.0), 0.0, 0.0, 0.0};
    current[axis + 1] = std::sin(rate * kDt / 2.0);
    const auto omega = AngularVelocityFromQuaternions(kIdentity, current, kDt);
    for (size_t component = 0; component < 3; ++component) {
      EXPECT_NEAR(omega[component], component == axis ? rate : 0.0, 1e-20);
    }
  }
}

TEST(QuaternionAngularVelocity, nonpositive_dt_is_exactly_zero) {
  const auto current = FromEuler(0.1, -0.2, 0.3);
  for (const double dt : {0.0, -kDt}) {
    ExpectZero(AngularVelocityFromQuaternions(kIdentity, current, dt));
  }
}
