#include "CarlaRGBCameraPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaRGBCameraPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds_image;
    std::unique_ptr<DDSPublisherImpl> _dds_info;
    sensor_msgs::msg::Image _image {};
    sensor_msgs::msg::CameraInfo _info {};
    bool _info_init {false};
  };

  bool CarlaRGBCameraPublisher::HasBeenInitialized() const {
    return _impl->_info_init;
  }

  void CarlaRGBCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    _impl->_info = std::move(sensor_msgs::msg::CameraInfo(height, width, fov));
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _impl->_info_init = true;
  }

  bool CarlaRGBCameraPublisher::Init(const TopicConfig& image_config, const TopicConfig& info_config) {
    return InitImage(image_config) && InitInfo(info_config);
  }

  bool CarlaRGBCameraPublisher::InitImage(const TopicConfig& config) {
    _impl->_dds_image = CreateDDSPublisher("sensor_msgs::msg::Image");
    if (!_impl->_dds_image) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += config.suffix;
    if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds_image->Init(config, _name, topic_name, /*use_preallocated_realloc=*/true)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaRGBCameraPublisher::InitInfo(const TopicConfig& config) {
    _impl->_dds_info = CreateDDSPublisher("sensor_msgs::msg::CameraInfo");
    if (!_impl->_dds_info) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += config.suffix;
    if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds_info->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaRGBCameraPublisher::Publish() {
    return PublishImage() && PublishInfo();
  }

  bool CarlaRGBCameraPublisher::PublishImage() {
    return _impl->_dds_image->Write(&_impl->_image);
  }

  bool CarlaRGBCameraPublisher::PublishInfo() {
    return _impl->_dds_info->Write(&_impl->_info);
  }

  void CarlaRGBCameraPublisher::SetImageData(int32_t seconds, uint32_t nanoseconds, uint32_t height, uint32_t width, const uint8_t* data) {
    std::vector<uint8_t> vector_data;
    const size_t size = height * width * 4;
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);
    SetImageData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaRGBCameraPublisher::SetImageData(int32_t seconds, uint32_t nanoseconds, uint32_t height, uint32_t width, std::vector<uint8_t>&& data) {

    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);
    _impl->_image.header(header);

    _impl->_image.width(width);
    _impl->_image.height(height);
    _impl->_image.encoding("bgra8");
    _impl->_image.is_bigendian(0);
    _impl->_image.step(_impl->_image.width() * sizeof(uint8_t) * 4);
    _impl->_image.data(std::move(data));
  }

  void CarlaRGBCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);
    _impl->_info.header(header);
  }

  void CarlaRGBCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset(x_offset);
    roi.y_offset(y_offset);
    roi.height(height);
    roi.width(width);
    roi.do_rectify(do_rectify);
    _impl->_info.roi(roi);
  }

  CarlaRGBCameraPublisher::CarlaRGBCameraPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaRGBCameraPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaRGBCameraPublisher::~CarlaRGBCameraPublisher() = default;
  CarlaRGBCameraPublisher::CarlaRGBCameraPublisher(const CarlaRGBCameraPublisher&) = default;
  CarlaRGBCameraPublisher& CarlaRGBCameraPublisher::operator=(const CarlaRGBCameraPublisher&) = default;
  CarlaRGBCameraPublisher::CarlaRGBCameraPublisher(CarlaRGBCameraPublisher&&) = default;
  CarlaRGBCameraPublisher& CarlaRGBCameraPublisher::operator=(CarlaRGBCameraPublisher&&) = default;
}
}
