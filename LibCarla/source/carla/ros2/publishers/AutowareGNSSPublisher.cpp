#include "AutowareGNSSPublisher.h"

#include "carla/ros2/types/PosePubSubTypes.h"
#include "carla/ros2/publishers/AutowarePublisherBase.hpp"

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;
using erc = eprosima::fastrtps::types::ReturnCode_t;

class AutowareGNSSPublisher::Implementation
: public AutowarePublisherBase<geometry_msgs::msg::Pose, geometry_msgs::msg::PosePubSubType>
{
public:
  Implementation(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "")
  : AutowarePublisherBase(ros_name, parent, ros_topic_name) {}
  const char * type() const override { return "pose"; }
};


bool AutowareGNSSPublisher::Init(const TopicConfig& config) {
  return _impl->Init(config);
}

bool AutowareGNSSPublisher::Publish() {
  return _impl->Publish();
}

// TODO: Verify whether this layout is correct
void AutowareGNSSPublisher::SetData(const double* position_data, const double* orientation_data) {
  geometry_msgs::msg::Pose pose;

  pose.position().x(*position_data++);
  pose.position().y(*position_data++);
  pose.position().z(*position_data++);

  pose.orientation().x(*orientation_data++);
  pose.orientation().y(*orientation_data++);
  pose.orientation().z(*orientation_data++);
  pose.orientation().w(*orientation_data++);

  _impl->SetData(pose);
}

AutowareGNSSPublisher::AutowareGNSSPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
_impl(std::make_shared<Implementation>(ros_name, parent, ros_topic_name)) {
  _name = ros_name;
  _parent = parent;
  _topic_name = ros_topic_name;
}


AutowareGNSSPublisher::AutowareGNSSPublisher(const AutowareGNSSPublisher& other) {
  _frame_id = other._frame_id;
  _name = other._name;
  _parent = other._parent;
  _impl = other._impl;
}

AutowareGNSSPublisher& AutowareGNSSPublisher::operator=(const AutowareGNSSPublisher& other) {
  _frame_id = other._frame_id;
  _name = other._name;
  _parent = other._parent;
  _impl = other._impl;

  return *this;
}

AutowareGNSSPublisher::AutowareGNSSPublisher(AutowareGNSSPublisher&& other) {
  _frame_id = std::move(other._frame_id);
  _name = std::move(other._name);
  _parent = std::move(other._parent);
  _impl = std::move(other._impl);
}

AutowareGNSSPublisher& AutowareGNSSPublisher::operator=(AutowareGNSSPublisher&& other) {
  _frame_id = std::move(other._frame_id);
  _name = std::move(other._name);
  _parent = std::move(other._parent);
  _impl = std::move(other._impl);

  return *this;
}
}  // namespace carla
}  // namespace ros2
