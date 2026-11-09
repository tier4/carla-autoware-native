// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/publishers/AutowareLocalizationPublisher.h"

#include "carla/Logging.h"
#include "carla/ros2/middleware/QosProfile.h"
#include "carla/ros2/publishers/PublisherImpl.h"
#include "carla/ros2/types/msg/AccelWithCovarianceStamped.h"
#include "carla/ros2/types/msg/Odometry.h"
#include "carla/ros2/types/msg/TFMessage.h"

#include <array>
#include <cmath>

namespace carla {
namespace ros2 {

struct AutowareLocalizationOdometryTraits { using msg_type = msg::Odometry; };
struct AutowareLocalizationAccelTraits { using msg_type = msg::AccelWithCovarianceStamped; };
struct AutowareLocalizationTfTraits { using msg_type = msg::TFMessage; };

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Diagonal covariance used by tier4 for both the pose and the twist/accel:
// x/y 0.01, z 0.04, roll/pitch (or wx/wy) 0.01, yaw (or wz) 0.0001.
std::array<double, 36> DiagonalCovariance() {
  std::array<double, 36> c = {};
  c[0] = 0.01;
  c[7] = 0.01;
  c[14] = 0.04;
  c[21] = 0.01;
  c[28] = 0.01;
  c[35] = 0.0001;
  return c;
}

}  // namespace

class AutowareLocalizationPublisher::Impl {
public:
  PublisherImpl<AutowareLocalizationOdometryTraits> odometry;
  PublisherImpl<AutowareLocalizationAccelTraits> acceleration;
  PublisherImpl<AutowareLocalizationTfTraits> tf;

  bool has_previous{false};
  double previous_time{0.0};
  std::array<double, 6> previous_twist{};  // linear xyz, angular xyz (ROS frame)
};

AutowareLocalizationPublisher::AutowareLocalizationPublisher()
  : BasePublisher("rt/localization", "map"),
    _impl(std::make_shared<Impl>()) {
  const QosProfile qos = QosProfile::ReliableTransientLocal();
  bool ok = true;
  ok &= _impl->odometry.Init("rt/localization/kinematic_state", qos);
  ok &= _impl->acceleration.Init("rt/localization/acceleration", qos);
  ok &= _impl->tf.Init("rt/tf", qos);
  if (!ok) {
    log_error("AutowareLocalizationPublisher: failed to initialise one or more writers");
  }
}

AutowareLocalizationPublisher::~AutowareLocalizationPublisher() = default;

bool AutowareLocalizationPublisher::Write(
    std::int32_t seconds,
    std::uint32_t nanoseconds,
    const AutowareLocalizationConfig &config,
    double x, double y, double z, double yaw_degrees,
    double vel_x, double vel_y, double vel_z,
    double ang_vel_x, double ang_vel_y, double ang_vel_z) {
  // CARLA world -> Autoware map (tier4 5cf27b660): offset from the CARLA
  // anchor, rotate by -map_to_carla_xy_yaw, unscale, then reflect Y (CARLA is
  // left-handed) around the map anchor.
  const double scale =
      std::abs(config.map_to_carla_scale) > 1.0e-9 ? config.map_to_carla_scale : 1.0;
  const double d_carla_x = x - config.reference_carla_base_x;
  const double d_carla_y = y - config.reference_carla_base_y;
  const double c = std::cos(-config.map_to_carla_xy_yaw);
  const double s = std::sin(-config.map_to_carla_xy_yaw);
  const double d_reflected_map_x = (c * d_carla_x - s * d_carla_y) / scale;
  const double d_reflected_map_y = (s * d_carla_x + c * d_carla_y) / scale;
  const double map_x = config.reference_map_x + d_reflected_map_x;
  const double map_y = config.reference_map_y - d_reflected_map_y;
  const double map_z = config.reference_map_z + (z - config.reference_carla_base_z);
  const double map_yaw = NormalizeAngle(config.map_to_carla_yaw - yaw_degrees * kPi / 180.0);

  msg::Header map_header;
  map_header.stamp.sec = seconds;
  map_header.stamp.nanosec = nanoseconds;
  map_header.frame_id = "map";

  msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(map_yaw * 0.5);
  orientation.w = std::cos(map_yaw * 0.5);

  // Twist in base_link (ROS, right-handed): flip the CARLA Y components.
  const std::array<double, 6> twist = {
      vel_x, -vel_y, vel_z, ang_vel_x, -ang_vel_y, -ang_vel_z};

  auto *odom = _impl->odometry.GetMessage();
  odom->header = map_header;
  odom->child_frame_id = "base_link";
  odom->pose.pose.position.x = map_x;
  odom->pose.pose.position.y = map_y;
  odom->pose.pose.position.z = map_z;
  odom->pose.pose.orientation = orientation;
  odom->pose.covariance = DiagonalCovariance();
  odom->twist.twist.linear.x = twist[0];
  odom->twist.twist.linear.y = twist[1];
  odom->twist.twist.linear.z = twist[2];
  odom->twist.twist.angular.x = twist[3];
  odom->twist.twist.angular.y = twist[4];
  odom->twist.twist.angular.z = twist[5];
  odom->twist.covariance = DiagonalCovariance();

  // Acceleration: finite difference of the published twist.
  std::array<double, 6> accel = {};
  const double now = static_cast<double>(seconds) + 1.0e-9 * static_cast<double>(nanoseconds);
  if (_impl->has_previous) {
    const double dt = now - _impl->previous_time;
    if (dt > 1.0e-6 && std::isfinite(dt)) {
      for (std::size_t i = 0; i < accel.size(); ++i) {
        accel[i] = (twist[i] - _impl->previous_twist[i]) / dt;
      }
    }
  }
  _impl->has_previous = true;
  _impl->previous_time = now;
  _impl->previous_twist = twist;

  auto *acc = _impl->acceleration.GetMessage();
  acc->header = map_header;
  acc->header.frame_id = "base_link";
  acc->accel.accel.linear.x = accel[0];
  acc->accel.accel.linear.y = accel[1];
  acc->accel.accel.linear.z = accel[2];
  acc->accel.accel.angular.x = accel[3];
  acc->accel.accel.angular.y = accel[4];
  acc->accel.accel.angular.z = accel[5];
  acc->accel.covariance = DiagonalCovariance();

  msg::TransformStamped transform;
  transform.header = map_header;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = map_x;
  transform.transform.translation.y = map_y;
  transform.transform.translation.z = map_z;
  transform.transform.rotation = orientation;
  auto *tf = _impl->tf.GetMessage();
  tf->transforms.assign(1, transform);
  return true;
}

bool AutowareLocalizationPublisher::Publish() {
  bool ok = true;
  ok &= _impl->odometry.Publish();
  ok &= _impl->acceleration.Publish();
  ok &= _impl->tf.Publish();
  return ok;
}

}  // namespace ros2
}  // namespace carla
