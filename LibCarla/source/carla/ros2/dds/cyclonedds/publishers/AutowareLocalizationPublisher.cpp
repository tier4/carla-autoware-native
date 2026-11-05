// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "AutowareLocalizationPublisher.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "dds/dds.h"
#include "AccelWithCovarianceStamped.h"
#include "Odometry.h"
#include "TFMessage.h"

namespace carla {
namespace ros2 {

namespace {

constexpr double Pi = 3.1415926535897932384626433832795;

double DegToRad(const double degrees)
{
  return degrees * Pi / 180.0;
}

double NormalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs_msg_Quaternion QuaternionFromYaw(const double yaw)
{
  geometry_msgs_msg_Quaternion quaternion {};
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

void SetPlanarPoseCovariance(double covariance[36])
{
  std::memset(covariance, 0, sizeof(double) * 36);
  covariance[0] = 0.01;     // x
  covariance[7] = 0.01;     // y
  covariance[14] = 0.04;    // z
  covariance[21] = 0.01;    // roll
  covariance[28] = 0.01;    // pitch
  covariance[35] = 0.0001;  // yaw
}

void SetTwistCovariance(double covariance[36])
{
  std::memset(covariance, 0, sizeof(double) * 36);
  covariance[0] = 0.01;     // vx
  covariance[7] = 0.01;     // vy
  covariance[14] = 0.04;    // vz
  covariance[21] = 0.01;    // wx
  covariance[28] = 0.01;    // wy
  covariance[35] = 0.0001;  // wz
}

struct Writer
{
  dds_entity_t participant { 0 };
  dds_entity_t topic { 0 };
  dds_entity_t writer { 0 };

  bool Init(
      const DomainId domain_id,
      const dds_topic_descriptor_t *desc,
      const char *topic_name,
      const char *human_name)
  {
    participant = dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr);
    if (participant < 0) {
      std::cerr << "CycloneDDS: Failed to create DomainParticipant for "
                << human_name << std::endl;
      return false;
    }

    topic = dds_create_topic(participant, desc, topic_name, nullptr, nullptr);
    if (topic < 0) {
      std::cerr << "CycloneDDS: Failed to create Topic for " << human_name
                << ": " << dds_strretcode(-topic) << std::endl;
      Cleanup();
      return false;
    }

    dds_qos_t *qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

    writer = dds_create_writer(participant, topic, qos, nullptr);
    dds_delete_qos(qos);
    if (writer < 0) {
      std::cerr << "CycloneDDS: Failed to create DataWriter for "
                << human_name << std::endl;
      Cleanup();
      return false;
    }
    return true;
  }

  bool Write(const void *sample) const
  {
    return writer > 0 && dds_write(writer, sample) >= 0;
  }

  void Cleanup()
  {
    if (participant > 0) {
      dds_delete(participant);
    }
    participant = 0;
    topic = 0;
    writer = 0;
  }
};

} // namespace

class AutowareLocalizationPublisher::Implementation
{
public:
  Writer odometry_writer {};
  Writer acceleration_writer {};
  Writer tf_writer {};

  nav_msgs_msg_Odometry odometry {};
  geometry_msgs_msg_AccelWithCovarianceStamped acceleration {};
  tf2_msgs_msg_TFMessage tf {};

  std::string map_frame { "map" };
  std::string base_link_frame { "base_link" };
  std::vector<geometry_msgs_msg_TransformStamped> tf_store;

  bool has_previous_velocity { false };
  double previous_time { 0.0 };
  double previous_linear_x { 0.0 };
  double previous_linear_y { 0.0 };
  double previous_linear_z { 0.0 };
  double previous_angular_x { 0.0 };
  double previous_angular_y { 0.0 };
  double previous_angular_z { 0.0 };

  ~Implementation()
  {
    odometry_writer.Cleanup();
    acceleration_writer.Cleanup();
    tf_writer.Cleanup();
  }
};

AutowareLocalizationPublisher::AutowareLocalizationPublisher(const DomainId domain_id)
  : _impl(std::make_shared<Implementation>())
{
  _impl->odometry_writer.Init(
      domain_id,
      &nav_msgs_msg_Odometry_desc,
      "rt/localization/kinematic_state",
      "Autoware localization odometry");

  _impl->acceleration_writer.Init(
      domain_id,
      &geometry_msgs_msg_AccelWithCovarianceStamped_desc,
      "rt/localization/acceleration",
      "Autoware localization acceleration");

  _impl->tf_writer.Init(
      domain_id,
      &tf2_msgs_msg_TFMessage_desc,
      "rt/tf",
      "Autoware map to base_link TF");
}

AutowareLocalizationPublisher::~AutowareLocalizationPublisher() = default;

void AutowareLocalizationPublisher::SetData(
    const int32_t seconds,
    const uint32_t nanoseconds,
    const AutowareLocalizationConfig &config,
    const AutowareLocalizationPose &base_link_pose,
    const AutowareLocalizationStatus &status)
{
  const double scale = std::abs(config.map_to_carla_scale) > 1.0e-9
      ? config.map_to_carla_scale
      : 1.0;

  const double d_carla_x = base_link_pose.x - config.reference_carla_base_x;
  const double d_carla_y = base_link_pose.y - config.reference_carla_base_y;

  const double c = std::cos(-config.map_to_carla_xy_yaw);
  const double s = std::sin(-config.map_to_carla_xy_yaw);
  const double d_reflected_map_x = (c * d_carla_x - s * d_carla_y) / scale;
  const double d_reflected_map_y = (s * d_carla_x + c * d_carla_y) / scale;

  const double map_x = config.reference_map_x + d_reflected_map_x;
  const double map_y = config.reference_map_y - d_reflected_map_y;
  const double map_z =
      config.reference_map_z + (base_link_pose.z - config.reference_carla_base_z);
  const double map_yaw = NormalizeAngle(
      config.map_to_carla_yaw - DegToRad(base_link_pose.yaw_degrees));

  builtin_interfaces_msg_Time time {};
  time.sec = seconds;
  time.nanosec = nanoseconds;

  std_msgs_msg_Header map_header {};
  map_header.stamp = time;
  map_header.frame_id = const_cast<char*>(_impl->map_frame.c_str());

  geometry_msgs_msg_Quaternion orientation = QuaternionFromYaw(map_yaw);

  _impl->odometry.header = map_header;
  _impl->odometry.child_frame_id = const_cast<char*>(_impl->base_link_frame.c_str());
  _impl->odometry.pose.pose.position.x = map_x;
  _impl->odometry.pose.pose.position.y = map_y;
  _impl->odometry.pose.pose.position.z = map_z;
  _impl->odometry.pose.pose.orientation = orientation;

  _impl->odometry.twist.twist.linear.x = status.vel_x_mps;
  _impl->odometry.twist.twist.linear.y = -status.vel_y_mps;
  _impl->odometry.twist.twist.linear.z = status.vel_z_mps;
  _impl->odometry.twist.twist.angular.x = status.ang_vel_x_radps;
  _impl->odometry.twist.twist.angular.y = -status.ang_vel_y_radps;
  _impl->odometry.twist.twist.angular.z = -status.ang_vel_z_radps;
  SetPlanarPoseCovariance(_impl->odometry.pose.covariance);
  SetTwistCovariance(_impl->odometry.twist.covariance);

  std_msgs_msg_Header base_link_header {};
  base_link_header.stamp = time;
  base_link_header.frame_id = const_cast<char*>(_impl->base_link_frame.c_str());

  const double current_time = static_cast<double>(seconds) +
      static_cast<double>(nanoseconds) * 1.0e-9;
  const double linear_x = _impl->odometry.twist.twist.linear.x;
  const double linear_y = _impl->odometry.twist.twist.linear.y;
  const double linear_z = _impl->odometry.twist.twist.linear.z;
  const double angular_x = _impl->odometry.twist.twist.angular.x;
  const double angular_y = _impl->odometry.twist.twist.angular.y;
  const double angular_z = _impl->odometry.twist.twist.angular.z;

  geometry_msgs_msg_Accel accel {};
  if (_impl->has_previous_velocity) {
    const double dt = current_time - _impl->previous_time;
    if (dt > 1.0e-6 && std::isfinite(dt)) {
      accel.linear.x = (linear_x - _impl->previous_linear_x) / dt;
      accel.linear.y = (linear_y - _impl->previous_linear_y) / dt;
      accel.linear.z = (linear_z - _impl->previous_linear_z) / dt;
      accel.angular.x = (angular_x - _impl->previous_angular_x) / dt;
      accel.angular.y = (angular_y - _impl->previous_angular_y) / dt;
      accel.angular.z = (angular_z - _impl->previous_angular_z) / dt;
    }
  }

  _impl->previous_time = current_time;
  _impl->previous_linear_x = linear_x;
  _impl->previous_linear_y = linear_y;
  _impl->previous_linear_z = linear_z;
  _impl->previous_angular_x = angular_x;
  _impl->previous_angular_y = angular_y;
  _impl->previous_angular_z = angular_z;
  _impl->has_previous_velocity = true;

  _impl->acceleration.header = base_link_header;
  _impl->acceleration.accel.accel = accel;
  SetTwistCovariance(_impl->acceleration.accel.covariance);

  geometry_msgs_msg_TransformStamped transform {};
  transform.header = map_header;
  transform.child_frame_id = const_cast<char*>(_impl->base_link_frame.c_str());
  transform.transform.translation.x = map_x;
  transform.transform.translation.y = map_y;
  transform.transform.translation.z = map_z;
  transform.transform.rotation = orientation;

  _impl->tf_store = {transform};
  _impl->tf.transforms._buffer = _impl->tf_store.data();
  _impl->tf.transforms._length = 1;
  _impl->tf.transforms._maximum = 1;
  _impl->tf.transforms._release = false;
}

bool AutowareLocalizationPublisher::Publish()
{
  bool ok = true;
  ok &= _impl->odometry_writer.Write(&_impl->odometry);
  ok &= _impl->acceleration_writer.Write(&_impl->acceleration);
  ok &= _impl->tf_writer.Write(&_impl->tf);
  return ok;
}

} // namespace ros2
} // namespace carla
