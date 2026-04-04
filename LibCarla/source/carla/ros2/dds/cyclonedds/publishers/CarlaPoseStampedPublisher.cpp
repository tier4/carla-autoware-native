#include "CarlaPoseStampedPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "PoseStamped.h"

namespace carla {
namespace ros2 {

class CarlaPoseStampedPublisher::Implementation
{
public:
  Implementation() = default;

  std::unique_ptr<DDSPublisherImpl> _dds;
  geometry_msgs_msg_PoseStamped _msg {};
  std::string _frame_id_store;
};

bool CarlaPoseStampedPublisher::Init(const TopicConfig& config) {
  _impl->_dds = CreateDDSPublisher("geometry_msgs_msg_PoseStamped");
  if (!_impl->_dds) return false;

  const std::string base { "rt/carla/" };
  std::string topic_name = base;
  if (!_parent.empty())
    topic_name += _parent + "/";
  topic_name += _name;
  topic_name += config.suffix;
  if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
    topic_name = custom_topic_name.value();
  }

  if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
    return false;
  }
  _frame_id = _name;
  return true;
}

bool CarlaPoseStampedPublisher::Publish() {
  return _impl->_dds->Write(&_impl->_msg);
}

void CarlaPoseStampedPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const double* position_data, const double* orientation_data) {
  builtin_interfaces_msg_Time time;
  time.sec = seconds;
  time.nanosec = nanoseconds;

  _impl->_frame_id_store = _frame_id;
  std_msgs_msg_Header header;
  header.stamp = time;
  header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

  geometry_msgs_msg_Pose pose;
  // TODO: Verify whether this layout is correct
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
  _topic_name = ros_topic_name;
}

CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(const CarlaPoseStampedPublisher&) = default;
CarlaPoseStampedPublisher& CarlaPoseStampedPublisher::operator=(const CarlaPoseStampedPublisher&) = default;
CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(CarlaPoseStampedPublisher&&) = default;
CarlaPoseStampedPublisher& CarlaPoseStampedPublisher::operator=(CarlaPoseStampedPublisher&&) = default;
}  // namespace ros2
}  // namespace carla
