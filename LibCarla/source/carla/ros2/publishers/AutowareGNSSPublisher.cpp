#include "AutowareGNSSPublisher.h"

#include "carla/ros2/types/PosePubSubTypes.h"
#include "carla/ros2/types/PoseWithCovarianceStampedPubSubTypes.h"
#include "carla/ros2/publishers/AutowarePublisherBase.hpp"

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;
using erc = eprosima::fastrtps::types::ReturnCode_t;

class PosePublisher
: public AutowarePublisherBase<geometry_msgs::msg::Pose, geometry_msgs::msg::PosePubSubType>
{
public:
  PosePublisher(const char* ros_name, const char* parent, const char* ros_topic_name)
  : AutowarePublisherBase(ros_name, parent, ros_topic_name) {}

  const char * type() const override { return "pose"; }
};

class PoseWithCovarianceStampedPublisher
: public AutowarePublisherBase<geometry_msgs::msg::PoseWithCovarianceStamped, geometry_msgs::msg::PoseWithCovarianceStampedPubSubType>
{
public:
  PoseWithCovarianceStampedPublisher(const char* ros_name, const char* parent, const char* ros_topic_name)
  : AutowarePublisherBase(ros_name, parent, ros_topic_name) {}

  const char * type() const override { return "pose with covariance stamped"; }
};

class AutowareGNSSPublisher::Implementation
{
public:
  Implementation() = delete;
  Implementation(const char* ros_name = "", const char* parent = "", const char* ros_topic_name = "")
  : _pose_publisher(ros_name, parent, ros_topic_name)
  , _pose_with_covariance_publisher(ros_name, parent, ros_topic_name) {}

  PosePublisher _pose_publisher;
  PoseWithCovarianceStampedPublisher _pose_with_covariance_publisher;
};

bool AutowareGNSSPublisher::Init(const TopicConfig& pose_config, const TopicConfig& pose_config_with_covariance_stamped) {
  return _impl->_pose_publisher.Init(pose_config) &&
    _impl->_pose_with_covariance_publisher.Init(pose_config_with_covariance_stamped);
}

bool AutowareGNSSPublisher::Publish() {
  return _impl->_pose_publisher.Publish() &&
    _impl->_pose_with_covariance_publisher.Publish();
}

void AutowareGNSSPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const double* position_data, const double* orientation_data) {
  geometry_msgs::msg::Pose pose;

  // TODO: Verify whether this data layout is correct
  pose.position().x(*position_data++);
  pose.position().y(*position_data++);
  pose.position().z(*position_data++);

  pose.orientation().x(*orientation_data++);
  pose.orientation().y(*orientation_data++);
  pose.orientation().z(*orientation_data++);
  pose.orientation().w(*orientation_data++);

  _impl->_pose_publisher.SetData(pose);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;

  builtin_interfaces::msg::Time time;
  time.sec(seconds);
  time.nanosec(nanoseconds);

  std_msgs::msg::Header header;
  header.stamp(std::move(time));
  header.frame_id(_impl->_pose_with_covariance_publisher.frame_id());

  std::array<double, 36> covariance{};  // TODO: Add some covariance matrix

  pose_with_covariance.header(std::move(header));
  pose_with_covariance.pose().pose(std::move(pose));
  pose_with_covariance.pose().covariance(std::move(covariance));

  _impl->_pose_with_covariance_publisher.SetData(pose_with_covariance);
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
