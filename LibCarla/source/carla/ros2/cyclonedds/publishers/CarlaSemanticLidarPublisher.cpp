#include "CarlaSemanticLidarPublisher.h"

#include <string>
#include <cstring>
#include <vector>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaSemanticLidarPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_PointCloud2 _lidar {};
    std::string _frame_id_store;
    std::vector<uint8_t> _data_store;
    std::vector<sensor_msgs_msg_PointField> _fields_store;
  };

  bool CarlaSemanticLidarPublisher::Init() {
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

  bool CarlaSemanticLidarPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_lidar) >= 0;
  }

  void CarlaSemanticLidarPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t elements, size_t height, size_t width, float* data) {
    float* it = data;
    float* end = &data[height * width * elements];
    for (++it; it < end; it += elements) {
        *it *= -1.0f;
    }

    std::vector<uint8_t> vector_data;
    const size_t size = height * width * sizeof(float) * elements;
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);
    SetData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaSemanticLidarPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
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
    descriptor4.name = const_cast<char*>("cos_inc_angle");
    descriptor4.offset = 12;
    descriptor4.datatype = 7;
    descriptor4.count = 1;
    sensor_msgs_msg_PointField descriptor5;
    descriptor5.name = const_cast<char*>("object_idx");
    descriptor5.offset = 16;
    descriptor5.datatype = 6;
    descriptor5.count = 1;
    sensor_msgs_msg_PointField descriptor6;
    descriptor6.name = const_cast<char*>("object_tag");
    descriptor6.offset = 20;
    descriptor6.datatype = 6;
    descriptor6.count = 1;

    const size_t point_size = 6 * sizeof(float);
    _impl->_lidar.header = header;
    _impl->_lidar.width = width;
    _impl->_lidar.height = height;
    _impl->_lidar.is_bigendian = false;
    _impl->_fields_store = {descriptor1, descriptor2, descriptor3, descriptor4, descriptor5, descriptor6};
    _impl->_lidar.fields._buffer = _impl->_fields_store.data();
    _impl->_lidar.fields._length = static_cast<uint32_t>(_impl->_fields_store.size());
    _impl->_lidar.fields._maximum = static_cast<uint32_t>(_impl->_fields_store.size());
    _impl->_lidar.fields._release = false;
    _impl->_lidar.point_step = point_size;
    _impl->_lidar.row_step = width * point_size;
    _impl->_lidar.is_dense = false;
    _impl->_data_store = std::move(data);
    _impl->_lidar.data._buffer = _impl->_data_store.data();
    _impl->_lidar.data._length = static_cast<uint32_t>(_impl->_data_store.size());
    _impl->_lidar.data._maximum = static_cast<uint32_t>(_impl->_data_store.size());
    _impl->_lidar.data._release = false;
  }

  CarlaSemanticLidarPublisher::CarlaSemanticLidarPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaSemanticLidarPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaSemanticLidarPublisher::~CarlaSemanticLidarPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaSemanticLidarPublisher::CarlaSemanticLidarPublisher(const CarlaSemanticLidarPublisher&) = default;
  CarlaSemanticLidarPublisher& CarlaSemanticLidarPublisher::operator=(const CarlaSemanticLidarPublisher&) = default;
  CarlaSemanticLidarPublisher::CarlaSemanticLidarPublisher(CarlaSemanticLidarPublisher&&) = default;
  CarlaSemanticLidarPublisher& CarlaSemanticLidarPublisher::operator=(CarlaSemanticLidarPublisher&&) = default;
}
}
