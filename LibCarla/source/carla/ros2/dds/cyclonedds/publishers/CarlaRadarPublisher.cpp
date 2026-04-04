// *** AUTO-GENERATED from FastDDS source -- DO NOT EDIT without reviewing ***
// Lines marked MANUAL_FIX need hand-editing for CycloneDDS compatibility.
// See tools/generate_cyclonedds_publishers.py for conversion rules.

#include "CarlaRadarPublisher.h"

#include <string>

#include "carla/sensor/data/RadarData.h"
#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaRadarPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    sensor_msgs_msg_PointCloud2 _radar {};
  };

  struct RadarDetectionWithPosition {
    float x;
    float y;
    float z;
    carla::sensor::data::RadarDetection detection;
  };

  bool CarlaRadarPublisher::Init(const TopicConfig& config) {
    _impl->_dds = CreateDDSPublisher("sensor_msgs_msg_PointCloud2");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    if (const auto custom_topic_name = ValidTopicName()) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/true)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaRadarPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_radar);
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

    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = _frame_id;

    sensor_msgs_msg_PointField descriptor1;
    descriptor1.name = "x";
    descriptor1.offset = 0;
    descriptor1.datatype = 7;
    descriptor1.count = 1;
    sensor_msgs_msg_PointField descriptor2;
    descriptor2.name = "y";
    descriptor2.offset = 4;
    descriptor2.datatype = 7;
    descriptor2.count = 1;
    sensor_msgs_msg_PointField descriptor3;
    descriptor3.name = "z";
    descriptor3.offset = 8;
    descriptor3.datatype = 7;
    descriptor3.count = 1;
    sensor_msgs_msg_PointField descriptor4;
    descriptor4.name = "velocity";
    descriptor4.offset = 12;
    descriptor4.datatype = 7;
    descriptor4.count = 1;
    sensor_msgs_msg_PointField descriptor5;
    descriptor5.name = "azimuth";
    descriptor5.offset = 16;
    descriptor5.datatype = 7;
    descriptor5.count = 1;
    sensor_msgs_msg_PointField descriptor6;
    descriptor6.name = "altitude";
    descriptor6.offset = 20;
    descriptor6.datatype = 7;
    descriptor6.count = 1;
    sensor_msgs_msg_PointField descriptor7;
    descriptor7.name = "depth";
    descriptor7.offset = 24;
    descriptor7.datatype = 7;
    descriptor7.count = 1;

    const size_t point_size = sizeof(RadarDetectionWithPosition);
    _impl->_radar.header = header;
    _impl->_radar.width = elements;
    _impl->_radar.height = height;
    _impl->_radar.is_bigendian = false;
    _impl->_radar.fields({descriptor1, descriptor2, descriptor3, descriptor4, descriptor5, descriptor6, descriptor7});  // MANUAL_FIX: sequence initializer list
    _impl->_radar.point_step = point_size;
    _impl->_radar.row_step = elements * point_size;
    _impl->_radar.is_dense = false;
    _impl->_radar.data = data;
  }

  CarlaRadarPublisher::CarlaRadarPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaRadarPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaRadarPublisher::~CarlaRadarPublisher() = default;
  CarlaRadarPublisher::CarlaRadarPublisher(const CarlaRadarPublisher&) = default;
  CarlaRadarPublisher& CarlaRadarPublisher::operator=(const CarlaRadarPublisher&) = default;
  CarlaRadarPublisher::CarlaRadarPublisher(CarlaRadarPublisher&&) = default;
  CarlaRadarPublisher& CarlaRadarPublisher::operator=(CarlaRadarPublisher&&) = default;
}
}
