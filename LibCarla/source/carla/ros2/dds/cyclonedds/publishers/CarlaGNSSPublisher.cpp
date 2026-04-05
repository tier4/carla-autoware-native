#include "CarlaGNSSPublisher.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "NavSatFix.h"

namespace carla {
namespace ros2 {

  struct CarlaGNSSPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_NavSatFix _nav {};
    std::string _frame_id_store;
  };

  bool CarlaGNSSPublisher::Init() {
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

    _impl->_topic = dds_create_topic(_impl->_participant, &sensor_msgs_msg_NavSatFix_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "Failed to create Topic" << std::endl;
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

  bool CarlaGNSSPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_nav) >= 0;
  }

  void CarlaGNSSPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const double* data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    std_msgs_msg_Header header;
    header.stamp = time;
    _impl->_frame_id_store = _frame_id;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    _impl->_nav.header = header;
    _impl->_nav.latitude = *data++;
    _impl->_nav.longitude = *data++;
    _impl->_nav.altitude = *data++;
  }

  CarlaGNSSPublisher::CarlaGNSSPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaGNSSPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaGNSSPublisher::~CarlaGNSSPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaGNSSPublisher::CarlaGNSSPublisher(const CarlaGNSSPublisher&) = default;
  CarlaGNSSPublisher& CarlaGNSSPublisher::operator=(const CarlaGNSSPublisher&) = default;
  CarlaGNSSPublisher::CarlaGNSSPublisher(CarlaGNSSPublisher&&) = default;
  CarlaGNSSPublisher& CarlaGNSSPublisher::operator=(CarlaGNSSPublisher&&) = default;
}
}
