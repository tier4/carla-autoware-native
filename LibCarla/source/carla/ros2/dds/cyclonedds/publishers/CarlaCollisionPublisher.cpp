#include "CarlaCollisionPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "CarlaCollisionEvent.h"

namespace carla {
namespace ros2 {

  struct CarlaCollisionPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    carla_msgs_msg_CarlaCollisionEvent _event {};
  };

  bool CarlaCollisionPublisher::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSPublisher("carla_msgs_msg_CarlaCollisionEvent");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    if (const auto custom_topic_name = ValidTopicName()) {
      topic_name = custom_topic_name.value();
    }

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaCollisionPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_event);
  }

  void CarlaCollisionPublisher::SetData(int32_t seconds, uint32_t nanoseconds, uint32_t actor_id, float x, float y, float z) {
    std::vector<float> vector_data ;
    SetData(seconds, nanoseconds, actor_id, {x, y, z});
  }

  void CarlaCollisionPublisher::SetData(int32_t seconds, uint32_t nanoseconds, uint32_t actor_id, std::vector<float>&& data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = _frame_id;

    geometry_msgs_msg_Vector3 impulse;
    impulse.x = data[0];
    impulse.y = data[1];
    impulse.z = data[2];
    _impl->_event.header = header;
    _impl->_event.other_actor_id = actor_id;
    _impl->_event.normal_impulse = impulse;
  }

  CarlaCollisionPublisher::CarlaCollisionPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaCollisionPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaCollisionPublisher::~CarlaCollisionPublisher() = default;
  CarlaCollisionPublisher::CarlaCollisionPublisher(const CarlaCollisionPublisher&) = default;
  CarlaCollisionPublisher& CarlaCollisionPublisher::operator=(const CarlaCollisionPublisher&) = default;
  CarlaCollisionPublisher::CarlaCollisionPublisher(CarlaCollisionPublisher&&) = default;
  CarlaCollisionPublisher& CarlaCollisionPublisher::operator=(CarlaCollisionPublisher&&) = default;
}
}
