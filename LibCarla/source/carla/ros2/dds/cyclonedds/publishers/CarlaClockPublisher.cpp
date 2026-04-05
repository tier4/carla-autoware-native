#include "CarlaClockPublisher.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "Clock.h"

namespace carla {
namespace ros2 {

  struct CarlaClockPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    rosgraph_msgs_msg_Clock _clock {};
  };

  bool CarlaClockPublisher::Init() {
    _impl->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string topic_name { "rt/clock" };
    _impl->_topic = dds_create_topic(_impl->_participant, &rosgraph_msgs_msg_Clock_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaClockPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_clock) >= 0;
  }

  void CarlaClockPublisher::SetData(int32_t sec, uint32_t nanosec) {
    _impl->_clock.clock.sec = sec;
    _impl->_clock.clock.nanosec = nanosec;
  }

  CarlaClockPublisher::CarlaClockPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaClockPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaClockPublisher::~CarlaClockPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaClockPublisher::CarlaClockPublisher(const CarlaClockPublisher&) = default;
  CarlaClockPublisher& CarlaClockPublisher::operator=(const CarlaClockPublisher&) = default;
  CarlaClockPublisher::CarlaClockPublisher(CarlaClockPublisher&&) = default;
  CarlaClockPublisher& CarlaClockPublisher::operator=(CarlaClockPublisher&&) = default;
}
}
