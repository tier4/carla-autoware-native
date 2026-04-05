// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaPoseStampedPublisher.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "PoseStamped.h"

#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"

namespace carla {
namespace ros2 {

class CarlaPoseStampedPublisher::Implementation
{
public:
  dds_entity_t _participant { 0 };
  dds_entity_t _topic { 0 };
  dds_entity_t _writer { 0 };
  geometry_msgs_msg_PoseStamped _msg {};
  std::string _frame_id_store;

  ~Implementation() {
    if (_participant > 0) dds_delete(_participant);
  }
};

bool CarlaPoseStampedPublisher::Init(DomainId domain_id) {
  const std::string base { "rt/carla/" };
  std::string topic_name = base;
  if (!_parent.empty())
    topic_name += _parent + "/";
  topic_name += _name;
  topic_name = SanitizeTopicName(topic_name);

  _impl->_participant = dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr);
  if (_impl->_participant < 0) {
    std::cerr << "CycloneDDS: Failed to create DomainParticipant for PoseStamped" << std::endl;
    return false;
  }

  _impl->_topic = dds_create_topic(_impl->_participant, &geometry_msgs_msg_PoseStamped_desc, topic_name.c_str(), nullptr, nullptr);
  if (_impl->_topic < 0) {
    std::cerr << "CycloneDDS: Failed to create Topic for PoseStamped: " << dds_strretcode(-_impl->_topic) << std::endl;
    dds_delete(_impl->_participant);
    _impl->_participant = 0;
    return false;
  }

  _impl->_writer = dds_create_writer(_impl->_participant, _impl->_topic, nullptr, nullptr);
  if (_impl->_writer < 0) {
    std::cerr << "CycloneDDS: Failed to create DataWriter for PoseStamped" << std::endl;
    dds_delete(_impl->_participant);
    _impl->_participant = 0;
    return false;
  }

  _frame_id = _name;
  return true;
}

bool CarlaPoseStampedPublisher::Publish() {
  return dds_write(_impl->_writer, &_impl->_msg) >= 0;
}

void CarlaPoseStampedPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const double* position_data, const double* orientation_data) {
  builtin_interfaces_msg_Time time;
  time.sec = seconds;
  time.nanosec = nanoseconds;

  std_msgs_msg_Header header {};
  header.stamp = time;
  _impl->_frame_id_store = _frame_id;
  header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

  geometry_msgs_msg_Pose pose {};
  pose.position.x = *position_data++;
  pose.position.y = *position_data++;
  pose.position.z = *position_data++;

  pose.orientation.x = *orientation_data++;
  pose.orientation.y = *orientation_data++;
  pose.orientation.z = *orientation_data++;
  pose.orientation.w = *orientation_data++;

  _impl->_msg.header = header;
  _impl->_msg.pose = pose;
}

CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
_impl(std::make_shared<Implementation>()) {
  _name = ros_name;
  _parent = parent;
}

CarlaPoseStampedPublisher::~CarlaPoseStampedPublisher() = default;
CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(const CarlaPoseStampedPublisher&) = default;
CarlaPoseStampedPublisher& CarlaPoseStampedPublisher::operator=(const CarlaPoseStampedPublisher&) = default;
CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(CarlaPoseStampedPublisher&&) = default;
CarlaPoseStampedPublisher& CarlaPoseStampedPublisher::operator=(CarlaPoseStampedPublisher&&) = default;
}  // namespace ros2
}  // namespace carla
