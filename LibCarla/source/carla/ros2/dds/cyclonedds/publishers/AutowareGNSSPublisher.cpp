#include "AutowareGNSSPublisher.h"

#include <string>
#include <cstring>
#include <cmath>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Pose.h"
#include "PoseWithCovarianceStamped.h"

namespace carla {
namespace ros2 {

class AutowareGNSSPublisher::Implementation
{
public:
  Implementation() = default;

  std::unique_ptr<DDSPublisherImpl> _pose_dds;
  geometry_msgs_msg_Pose _pose {};
  std::string _pose_frame_id;

  std::unique_ptr<DDSPublisherImpl> _pose_with_covariance_dds;
  geometry_msgs_msg_PoseWithCovarianceStamped _pose_with_covariance {};
  std::string _pose_with_covariance_frame_id;
  std::string _frame_id_store;
};

bool AutowareGNSSPublisher::Init(const TopicConfig& pose_config, const TopicConfig& pose_config_with_covariance_stamped) {
  // Init pose publisher
  _impl->_pose_dds = CreateDDSPublisher("geometry_msgs_msg_Pose");
  if (!_impl->_pose_dds) return false;

  {
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += pose_config.suffix;
    if (const auto custom_topic_name = ValidTopicName(pose_config.suffix)) {
      topic_name = custom_topic_name.value();
    }
    if (!_impl->_pose_dds->Init(pose_config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _impl->_pose_frame_id = _name;
  }

  // Init pose with covariance publisher
  _impl->_pose_with_covariance_dds = CreateDDSPublisher("geometry_msgs_msg_PoseWithCovarianceStamped");
  if (!_impl->_pose_with_covariance_dds) return false;

  {
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += pose_config_with_covariance_stamped.suffix;
    if (const auto custom_topic_name = ValidTopicName(pose_config_with_covariance_stamped.suffix)) {
      topic_name = custom_topic_name.value();
    }
    if (!_impl->_pose_with_covariance_dds->Init(pose_config_with_covariance_stamped, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _impl->_pose_with_covariance_frame_id = _name;
  }

  _frame_id = _name;
  return true;
}

bool AutowareGNSSPublisher::Publish() {
  return _impl->_pose_dds->Write(&_impl->_pose) &&
    _impl->_pose_with_covariance_dds->Write(&_impl->_pose_with_covariance);
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

  // TODO: Verify whether this data layout is correct

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

  geometry_msgs_msg_PoseWithCovarianceStamped pose_with_covariance;

  builtin_interfaces_msg_Time time;
  time.sec = seconds;
  time.nanosec = nanoseconds;

  _impl->_frame_id_store = _impl->_pose_with_covariance_frame_id;
  std_msgs_msg_Header header;
  header.stamp = time;
  header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

  pose_with_covariance.header = header;
  pose_with_covariance.pose.pose = pose;
  std::memset(pose_with_covariance.pose.covariance, 0, sizeof(pose_with_covariance.pose.covariance));  // TODO: Add some covariance matrix

  _impl->_pose_with_covariance = pose_with_covariance;
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
  _topic_name = ros_topic_name;
}

AutowareGNSSPublisher::AutowareGNSSPublisher(const AutowareGNSSPublisher&) = default;
AutowareGNSSPublisher& AutowareGNSSPublisher::operator=(const AutowareGNSSPublisher&) = default;
AutowareGNSSPublisher::AutowareGNSSPublisher(AutowareGNSSPublisher&&) = default;
AutowareGNSSPublisher& AutowareGNSSPublisher::operator=(AutowareGNSSPublisher&&) = default;
}  // namespace ros2
}  // namespace carla
