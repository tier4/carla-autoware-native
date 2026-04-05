#include "CarlaCollisionPublisher.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/dds/cyclonedds/CycloneDDSTopicHelper.h"
#include "CarlaCollisionEvent.h"

namespace carla {
namespace ros2 {

  struct CarlaCollisionPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    carla_msgs_msg_CarlaCollisionEvent _event {};
    std::string _frame_id_store;
  };

  bool CarlaCollisionPublisher::Init() {
    _impl->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;

    topic_name = SanitizeTopicName(topic_name);
    _impl->_topic = dds_create_topic(_impl->_participant, &carla_msgs_msg_CarlaCollisionEvent_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    _impl->_writer = dds_create_writer(_impl->_participant, _impl->_topic, nullptr, nullptr);
    if (_impl->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool CarlaCollisionPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_event) >= 0;
  }

  void CarlaCollisionPublisher::SetData(int32_t seconds, uint32_t nanoseconds, uint32_t actor_id, float x, float y, float z) {
    std::vector<float> vector_data;
    SetData(seconds, nanoseconds, actor_id, {x, y, z});
  }

  void CarlaCollisionPublisher::SetData(int32_t seconds, uint32_t nanoseconds, uint32_t actor_id, std::vector<float>&& data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    std_msgs_msg_Header header;
    header.stamp = time;
    _impl->_frame_id_store = _frame_id;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    geometry_msgs_msg_Vector3 impulse;
    impulse.x = data[0];
    impulse.y = data[1];
    impulse.z = data[2];
    _impl->_event.header = header;
    _impl->_event.other_actor_id = actor_id;
    _impl->_event.normal_impulse = impulse;
  }

  CarlaCollisionPublisher::CarlaCollisionPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaCollisionPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaCollisionPublisher::~CarlaCollisionPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaCollisionPublisher::CarlaCollisionPublisher(const CarlaCollisionPublisher&) = default;
  CarlaCollisionPublisher& CarlaCollisionPublisher::operator=(const CarlaCollisionPublisher&) = default;
  CarlaCollisionPublisher::CarlaCollisionPublisher(CarlaCollisionPublisher&&) = default;
  CarlaCollisionPublisher& CarlaCollisionPublisher::operator=(CarlaCollisionPublisher&&) = default;
}
}
