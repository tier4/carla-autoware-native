#include "CarlaPoseStampedPublisher.h"

#include "PoseStampedPubSubTypes.h"
#include "carla/ros2/publishers/AutowarePublisherBase.hpp"

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;
using erc = eprosima::fastrtps::types::ReturnCode_t;

class CarlaPoseStampedPublisher::Implementation
: public AutowarePublisherBase<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStampedPubSubType>
{
public:
  Implementation(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "")
  : AutowarePublisherBase(ros_name, parent, ros_topic_name) {}

  const char * type() const override { return "pose stamped"; }
};


bool CarlaPoseStampedPublisher::Init(const TopicConfig& config) {
  return _impl->Init(config);
}

bool CarlaPoseStampedPublisher::Publish() {
  return _impl->Publish();
}

void CarlaPoseStampedPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const double* position_data, const double* orientation_data) {
  builtin_interfaces::msg::Time time;
  time.sec(seconds);
  time.nanosec(nanoseconds);

  std_msgs::msg::Header header;
  header.stamp(std::move(time));
  header.frame_id(_impl->frame_id());

  geometry_msgs::msg::Pose pose;
  // TODO: Verify whether this layout is correct
  pose.position().x(*position_data++);
  pose.position().y(*position_data++);
  pose.position().z(*position_data++);

  pose.orientation().x(*orientation_data++);
  pose.orientation().y(*orientation_data++);
  pose.orientation().z(*orientation_data++);
  pose.orientation().w(*orientation_data++);

  geometry_msgs::msg::PoseStamped msg;
  msg.header(std::move(header));
  msg.pose(std::move(pose));

  _impl->SetData(msg);
}

CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
_impl(std::make_shared<Implementation>(ros_name, parent, ros_topic_name)) {
  _name = ros_name;
  _parent = parent;
  _topic_name = ros_topic_name;
}


CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(const CarlaPoseStampedPublisher& other) {
  _frame_id = other._frame_id;
  _name = other._name;
  _parent = other._parent;
  _impl = other._impl;
}

CarlaPoseStampedPublisher& CarlaPoseStampedPublisher::operator=(const CarlaPoseStampedPublisher& other) {
  _frame_id = other._frame_id;
  _name = other._name;
  _parent = other._parent;
  _impl = other._impl;

  return *this;
}

CarlaPoseStampedPublisher::CarlaPoseStampedPublisher(CarlaPoseStampedPublisher&& other) {
  _frame_id = std::move(other._frame_id);
  _name = std::move(other._name);
  _parent = std::move(other._parent);
  _impl = std::move(other._impl);
}

CarlaPoseStampedPublisher& CarlaPoseStampedPublisher::operator=(CarlaPoseStampedPublisher&& other) {
  _frame_id = std::move(other._frame_id);
  _name = std::move(other._name);
  _parent = std::move(other._parent);
  _impl = std::move(other._impl);

  return *this;
}
}  // namespace carla
}  // namespace ros2
