#include "CarlaRadarPublisher.h"

#include <string>
#include <vector>
#include <iostream>

#include "carla/sensor/data/RadarData.h"
#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaRadarPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_PointCloud2 _radar {};
    std::string _frame_id_store;
    std::vector<uint8_t> _data_store;
    std::vector<sensor_msgs_msg_PointField> _fields_store;
  };

  struct RadarDetectionWithPosition {
    float x;
    float y;
    float z;
    carla::sensor::data::RadarDetection detection;
  };

  bool CarlaRadarPublisher::Init() {
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
    _impl->_topic = dds_create_topic(_impl->_participant, &sensor_msgs_msg_PointCloud2_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaRadarPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_radar) >= 0;
  }

  void CarlaRadarPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, size_t elements, const uint8_t* data) {

    std::vector<uint8_t> vector_data;
    const size_t size = elements * sizeof(RadarDetectionWithPosition);
    vector_data.resize(size);
    RadarDetectionWithPosition* radar_data = (RadarDetectionWithPosition*)&vector_data[0];
    carla::sensor::data::RadarDetection* detection_data = (carla::sensor::data::RadarDetection*)data;
    for (size_t i = 0; i < elements; ++i, ++radar_data, ++detection_data) {
      radar_data->x = detection_data->depth * cosf(detection_data->azimuth) * cosf(-detection_data->altitude);
      radar_data->y = detection_data->depth * sinf(-detection_data->azimuth) * cosf(detection_data->altitude);
      radar_data->z = detection_data->depth * sinf(detection_data->altitude);
      radar_data->detection = *detection_data;
    }

    SetData(seconds, nanoseconds, height, width, elements, std::move(vector_data));
  }

  void CarlaRadarPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, size_t elements, std::vector<uint8_t>&& data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    sensor_msgs_msg_PointField descriptor1;
    descriptor1.name = const_cast<char*>("x");
    descriptor1.offset = 0;
    descriptor1.datatype = 7;
    descriptor1.count = 1;
    sensor_msgs_msg_PointField descriptor2;
    descriptor2.name = const_cast<char*>("y");
    descriptor2.offset = 4;
    descriptor2.datatype = 7;
    descriptor2.count = 1;
    sensor_msgs_msg_PointField descriptor3;
    descriptor3.name = const_cast<char*>("z");
    descriptor3.offset = 8;
    descriptor3.datatype = 7;
    descriptor3.count = 1;
    sensor_msgs_msg_PointField descriptor4;
    descriptor4.name = const_cast<char*>("velocity");
    descriptor4.offset = 12;
    descriptor4.datatype = 7;
    descriptor4.count = 1;
    sensor_msgs_msg_PointField descriptor5;
    descriptor5.name = const_cast<char*>("azimuth");
    descriptor5.offset = 16;
    descriptor5.datatype = 7;
    descriptor5.count = 1;
    sensor_msgs_msg_PointField descriptor6;
    descriptor6.name = const_cast<char*>("altitude");
    descriptor6.offset = 20;
    descriptor6.datatype = 7;
    descriptor6.count = 1;
    sensor_msgs_msg_PointField descriptor7;
    descriptor7.name = const_cast<char*>("depth");
    descriptor7.offset = 24;
    descriptor7.datatype = 7;
    descriptor7.count = 1;

    const size_t point_size = sizeof(RadarDetectionWithPosition);
    _impl->_radar.header = header;
    _impl->_radar.width = elements;
    _impl->_radar.height = height;
    _impl->_radar.is_bigendian = false;
    _impl->_fields_store = {descriptor1, descriptor2, descriptor3, descriptor4, descriptor5, descriptor6, descriptor7};
    _impl->_radar.fields._buffer = _impl->_fields_store.data();
    _impl->_radar.fields._length = static_cast<uint32_t>(_impl->_fields_store.size());
    _impl->_radar.fields._maximum = static_cast<uint32_t>(_impl->_fields_store.size());
    _impl->_radar.fields._release = false;
    _impl->_radar.point_step = point_size;
    _impl->_radar.row_step = elements * point_size;
    _impl->_radar.is_dense = false;
    _impl->_data_store = std::move(data);
    _impl->_radar.data._buffer = _impl->_data_store.data();
    _impl->_radar.data._length = static_cast<uint32_t>(_impl->_data_store.size());
    _impl->_radar.data._maximum = static_cast<uint32_t>(_impl->_data_store.size());
    _impl->_radar.data._release = false;
  }

  CarlaRadarPublisher::CarlaRadarPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaRadarPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaRadarPublisher::~CarlaRadarPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaRadarPublisher::CarlaRadarPublisher(const CarlaRadarPublisher&) = default;
  CarlaRadarPublisher& CarlaRadarPublisher::operator=(const CarlaRadarPublisher&) = default;
  CarlaRadarPublisher::CarlaRadarPublisher(CarlaRadarPublisher&&) = default;
  CarlaRadarPublisher& CarlaRadarPublisher::operator=(CarlaRadarPublisher&&) = default;
}
}
