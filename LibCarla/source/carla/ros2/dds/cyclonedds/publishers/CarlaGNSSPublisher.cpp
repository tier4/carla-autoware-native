#include "CarlaGNSSPublisher.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/dds/cyclonedds/CycloneDDSTopicHelper.h"
#include "carla/ros2/dds/cyclonedds/conversions.hpp"
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
    TopicConfig config;
    config.domain_id = GetDomainId();
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;
    return Init(config);
  }

  bool CarlaGNSSPublisher::Init(const TopicConfig& config) {
    _impl->_participant = dds_create_participant(config.domain_id, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;

    if (const auto custom_topic_name = ValidTopicName()) {
        topic_name = custom_topic_name.value();
    }
    topic_name = SanitizeTopicName(topic_name);
    _impl->_topic = dds_create_topic(_impl->_participant, &sensor_msgs_msg_NavSatFix_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    dds_qos_t* qos = dds_create_qos();
    configure_cyclone_qos(config, qos);
    _impl->_writer = dds_create_writer(_impl->_participant, _impl->_topic, qos, nullptr);
    dds_delete_qos(qos);
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

  CarlaGNSSPublisher::CarlaGNSSPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaGNSSPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
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
