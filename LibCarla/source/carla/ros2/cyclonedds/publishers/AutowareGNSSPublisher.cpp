// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "AutowareGNSSPublisher.h"

#include <string>
#include <cstring>
#include <cmath>
#include <iostream>

#include "dds/dds.h"
#include "Pose.h"
#include "PoseWithCovarianceStamped.h"

#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"

namespace carla {
namespace ros2 {

class AutowareGNSSPublisher::Implementation
{
public:
  // Pose publisher
  dds_entity_t _pose_participant { 0 };
  dds_entity_t _pose_topic { 0 };
  dds_entity_t _pose_writer { 0 };
  geometry_msgs_msg_Pose _pose {};
  std::string _pose_frame_id;

  // PoseWithCovarianceStamped publisher
  dds_entity_t _pwcs_participant { 0 };
  dds_entity_t _pwcs_topic { 0 };
  dds_entity_t _pwcs_writer { 0 };
  geometry_msgs_msg_PoseWithCovarianceStamped _pose_with_covariance {};
  std::string _pose_with_covariance_frame_id;

  ~Implementation() {
    if (_pose_participant > 0) dds_delete(_pose_participant);
    if (_pwcs_participant > 0) dds_delete(_pwcs_participant);
  }
};

bool AutowareGNSSPublisher::Init(DomainId domain_id) {
  // Build topic names
  const std::string base { "rt/carla/" };
  std::string pose_topic_name = base;
  if (!_parent.empty())
    pose_topic_name += _parent + "/";
  pose_topic_name += _name;
  pose_topic_name += "/pose";
  pose_topic_name = SanitizeTopicName(pose_topic_name);

  std::string pwcs_topic_name = base;
  if (!_parent.empty())
    pwcs_topic_name += _parent + "/";
  pwcs_topic_name += _name;
  pwcs_topic_name += "/pose_with_covariance";
  pwcs_topic_name = SanitizeTopicName(pwcs_topic_name);

  // QoS: RELIABLE, VOLATILE, KEEP_LAST(1)
  dds_qos_t* qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
  dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

  // Init pose publisher
  _impl->_pose_participant = dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr);
  if (_impl->_pose_participant < 0) {
    std::cerr << "CycloneDDS: Failed to create DomainParticipant for Autoware GNSS Pose" << std::endl;
    dds_delete_qos(qos);
    return false;
  }

  _impl->_pose_topic = dds_create_topic(_impl->_pose_participant, &geometry_msgs_msg_Pose_desc, pose_topic_name.c_str(), nullptr, nullptr);
  if (_impl->_pose_topic < 0) {
    std::cerr << "CycloneDDS: Failed to create Topic for Autoware GNSS Pose: " << dds_strretcode(-_impl->_pose_topic) << std::endl;
    dds_delete_qos(qos);
    return false;
  }

  _impl->_pose_writer = dds_create_writer(_impl->_pose_participant, _impl->_pose_topic, qos, nullptr);
  if (_impl->_pose_writer < 0) {
    std::cerr << "CycloneDDS: Failed to create DataWriter for Autoware GNSS Pose" << std::endl;
    dds_delete_qos(qos);
    return false;
  }

  _impl->_pose_frame_id = _name;

  // Init pose with covariance publisher
  _impl->_pwcs_participant = dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr);
  if (_impl->_pwcs_participant < 0) {
    std::cerr << "CycloneDDS: Failed to create DomainParticipant for Autoware GNSS PoseWithCovarianceStamped" << std::endl;
    dds_delete_qos(qos);
    return false;
  }

  _impl->_pwcs_topic = dds_create_topic(_impl->_pwcs_participant, &geometry_msgs_msg_PoseWithCovarianceStamped_desc, pwcs_topic_name.c_str(), nullptr, nullptr);
  if (_impl->_pwcs_topic < 0) {
    std::cerr << "CycloneDDS: Failed to create Topic for Autoware GNSS PoseWithCovarianceStamped: " << dds_strretcode(-_impl->_pwcs_topic) << std::endl;
    dds_delete_qos(qos);
    return false;
  }

  _impl->_pwcs_writer = dds_create_writer(_impl->_pwcs_participant, _impl->_pwcs_topic, qos, nullptr);
  dds_delete_qos(qos);
  if (_impl->_pwcs_writer < 0) {
    std::cerr << "CycloneDDS: Failed to create DataWriter for Autoware GNSS PoseWithCovarianceStamped" << std::endl;
    return false;
  }

  _impl->_pose_with_covariance_frame_id = _name;

  _frame_id = _name;
  return true;
}

bool AutowareGNSSPublisher::Publish() {
  bool ok = true;
  if (dds_write(_impl->_pose_writer, &_impl->_pose) < 0) ok = false;
  if (dds_write(_impl->_pwcs_writer, &_impl->_pose_with_covariance) < 0) ok = false;
  return ok;
}

void AutowareGNSSPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const float* translation, const float* rotation, const double* mgrs_offset_position)
{
  geometry_msgs_msg_Pose pose;

  /// @note: Copied from CarlaTransformPublisher
  const float tx = *translation++;
  const float ty = *translation++;
  const float tz = *translation++;

  const float rx = ((*rotation++) * -1.0f) * (M_PIf32 / 180.0f);
  const float ry = ((*rotation++) * -1.0f) * (M_PIf32 / 180.0f);
  const float rz = *rotation++ * (M_PIf32 / 180.0f);

  const float cr = cosf(rz * 0.5f);
  const float sr = sinf(rz * 0.5f);
  const float cp = cosf(rx * 0.5f);
  const float sp = sinf(rx * 0.5f);
  const float cy = cosf(ry * 0.5f);
  const float sy = sinf(ry * 0.5f);

  const double mgrs_x = mgrs_offset_position[0];
  const double mgrs_y = mgrs_offset_position[1];
  const double mgrs_z = mgrs_offset_position[2];

  pose.position.x = tx + mgrs_x;
  pose.position.y = -ty + mgrs_y; // note original y was negated
  pose.position.z = tz + mgrs_z;

  pose.orientation.w = cr * cp * cy + sr * sp * sy;
  pose.orientation.x = sr * cp * cy - cr * sp * sy;
  pose.orientation.y = cr * sp * cy + sr * cp * sy;
  pose.orientation.z = cr * cp * sy - sr * sp * cy;

  _impl->_pose = pose;

  geometry_msgs_msg_PoseWithCovarianceStamped pose_with_covariance {};

  builtin_interfaces_msg_Time time;
  time.sec = seconds;
  time.nanosec = nanoseconds;

  std_msgs_msg_Header header {};
  header.stamp = time;
  header.frame_id = const_cast<char*>(_impl->_pose_with_covariance_frame_id.c_str());

  pose_with_covariance.header = header;
  pose_with_covariance.pose.pose = pose;
  std::memset(pose_with_covariance.pose.covariance, 0, sizeof(pose_with_covariance.pose.covariance));  // TODO: Add some covariance matrix

  _impl->_pose_with_covariance = std::move(pose_with_covariance);
}

void AutowareGNSSPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const float* translation, const float* rotation)
{
  const double none_mgrs_offset[3] = {0, 0, 0}; // Do not apply mgrs offset, pass 0 values instead
  SetData(seconds, nanoseconds, translation, rotation, none_mgrs_offset);
}

AutowareGNSSPublisher::AutowareGNSSPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
_impl(std::make_shared<Implementation>()) {
  _name = ros_name;
  _parent = parent;
}

AutowareGNSSPublisher::~AutowareGNSSPublisher() = default;
AutowareGNSSPublisher::AutowareGNSSPublisher(const AutowareGNSSPublisher&) = default;
AutowareGNSSPublisher& AutowareGNSSPublisher::operator=(const AutowareGNSSPublisher&) = default;
AutowareGNSSPublisher::AutowareGNSSPublisher(AutowareGNSSPublisher&&) = default;
AutowareGNSSPublisher& AutowareGNSSPublisher::operator=(AutowareGNSSPublisher&&) = default;
}  // namespace ros2
}  // namespace carla
