#include "CarlaLineInvasionPublisher.h"

#include <string>
#include <vector>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/dds/cyclonedds/CycloneDDSTopicHelper.h"
#include "carla/ros2/dds/cyclonedds/conversions.hpp"
#include "CarlaLineInvasion.h"

namespace carla {
namespace ros2 {

  struct CarlaLineInvasionPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    carla_msgs_msg_CarlaLineInvasion _event {};
    std::string _frame_id_store;
    std::vector<int32_t> _crossed_lane_markings_store;
  };

  bool CarlaLineInvasionPublisher::Init() {
    TopicConfig config;
    config.domain_id = GetDomainId();
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;
    return Init(config);
  }

  bool CarlaLineInvasionPublisher::Init(const TopicConfig& config) {
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
    _impl->_topic = dds_create_topic(_impl->_participant, &carla_msgs_msg_CarlaLineInvasion_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaLineInvasionPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_event) >= 0;
  }

  void CarlaLineInvasionPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const int32_t* data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    _impl->_event.header = header;
    _impl->_crossed_lane_markings_store = {data[0], data[1], data[2]};
    _impl->_event.crossed_lane_markings._buffer = _impl->_crossed_lane_markings_store.data();
    _impl->_event.crossed_lane_markings._length = static_cast<uint32_t>(_impl->_crossed_lane_markings_store.size());
    _impl->_event.crossed_lane_markings._maximum = static_cast<uint32_t>(_impl->_crossed_lane_markings_store.size());
    _impl->_event.crossed_lane_markings._release = false;
  }

  CarlaLineInvasionPublisher::CarlaLineInvasionPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaLineInvasionPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaLineInvasionPublisher::~CarlaLineInvasionPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaLineInvasionPublisher::CarlaLineInvasionPublisher(const CarlaLineInvasionPublisher&) = default;
  CarlaLineInvasionPublisher& CarlaLineInvasionPublisher::operator=(const CarlaLineInvasionPublisher&) = default;
  CarlaLineInvasionPublisher::CarlaLineInvasionPublisher(CarlaLineInvasionPublisher&&) = default;
  CarlaLineInvasionPublisher& CarlaLineInvasionPublisher::operator=(CarlaLineInvasionPublisher&&) = default;
}
}
