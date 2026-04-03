#include "CarlaSemanticLidarPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaSemanticLidarPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    sensor_msgs::msg::PointCloud2 _lidar {};
  };

  bool CarlaSemanticLidarPublisher::Init(const TopicConfig& config) {
    _impl->_dds = CreateDDSPublisher("sensor_msgs::msg::PointCloud2");
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

  bool CarlaSemanticLidarPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_lidar);
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
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);

    sensor_msgs::msg::PointField descriptor1;
    descriptor1.name("x");
    descriptor1.offset(0);
    descriptor1.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor1.count(1);
    sensor_msgs::msg::PointField descriptor2;
    descriptor2.name("y");
    descriptor2.offset(4);
    descriptor2.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor2.count(1);
    sensor_msgs::msg::PointField descriptor3;
    descriptor3.name("z");
    descriptor3.offset(8);
    descriptor3.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor3.count(1);
    sensor_msgs::msg::PointField descriptor4;
    descriptor4.name("cos_inc_angle");
    descriptor4.offset(12);
    descriptor4.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor4.count(1);
    sensor_msgs::msg::PointField descriptor5;
    descriptor5.name("object_idx");
    descriptor5.offset(16);
    descriptor5.datatype(sensor_msgs::msg::PointField__UINT32);
    descriptor5.count(1);
    sensor_msgs::msg::PointField descriptor6;
    descriptor6.name("object_tag");
    descriptor6.offset(20);
    descriptor6.datatype(sensor_msgs::msg::PointField__UINT32);
    descriptor6.count(1);

    const size_t point_size = 6 * sizeof(float);
    _impl->_lidar.header(std::move(header));
    _impl->_lidar.width(width);
    _impl->_lidar.height(height);
    _impl->_lidar.is_bigendian(false);
    _impl->_lidar.fields({descriptor1, descriptor2, descriptor3, descriptor4, descriptor5, descriptor6});
    _impl->_lidar.point_step(point_size);
    _impl->_lidar.row_step(width * point_size);
    _impl->_lidar.is_dense(false);
    _impl->_lidar.data(std::move(data));
  }

  CarlaSemanticLidarPublisher::CarlaSemanticLidarPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaSemanticLidarPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaSemanticLidarPublisher::~CarlaSemanticLidarPublisher() = default;
  CarlaSemanticLidarPublisher::CarlaSemanticLidarPublisher(const CarlaSemanticLidarPublisher&) = default;
  CarlaSemanticLidarPublisher& CarlaSemanticLidarPublisher::operator=(const CarlaSemanticLidarPublisher&) = default;
  CarlaSemanticLidarPublisher::CarlaSemanticLidarPublisher(CarlaSemanticLidarPublisher&&) = default;
  CarlaSemanticLidarPublisher& CarlaSemanticLidarPublisher::operator=(CarlaSemanticLidarPublisher&&) = default;
}
}
