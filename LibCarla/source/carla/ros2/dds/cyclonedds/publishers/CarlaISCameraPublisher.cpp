// *** AUTO-GENERATED from FastDDS source -- DO NOT EDIT without reviewing ***
// Lines marked MANUAL_FIX need hand-editing for CycloneDDS compatibility.
// See tools/generate_cyclonedds_publishers.py for conversion rules.

#include "CarlaISCameraPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaISCameraPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds_image;
    std::unique_ptr<DDSPublisherImpl> _dds_info;
    sensor_msgs_msg_Image _image {};
    sensor_msgs_msg_CameraInfo _info {};
    bool _info_init {false};
  };

  bool CarlaISCameraPublisher::HasBeenInitialized() const {
    return _impl->_info_init;
  }

  void CarlaISCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    _impl->_info = std::move(sensor_msgs_msg_CameraInfo(height, width, fov));  // MANUAL_FIX: CameraInfo constructor
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _impl->_info_init = true;
  }

  bool CarlaISCameraPublisher::Init(const TopicConfig& config) {
    return InitImage(config) && InitInfo(config);
  }

  bool CarlaISCameraPublisher::InitImage(const TopicConfig& config) {
    _impl->_dds_image = CreateDDSPublisher("sensor_msgs_msg_Image");
    if (!_impl->_dds_image) return false;

    const std::string publisher_type {"/image"};
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;
    if (const auto custom_topic_name = ValidTopicName(publisher_type)) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds_image->Init(config, _name, topic_name, /*use_preallocated_realloc=*/true)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaISCameraPublisher::InitInfo(const TopicConfig& config) {
    _impl->_dds_info = CreateDDSPublisher("sensor_msgs_msg_CameraInfo");
    if (!_impl->_dds_info) return false;

    const std::string publisher_type {"/camera_info"};
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;
    if (const auto custom_topic_name = ValidTopicName(publisher_type)) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds_info->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaISCameraPublisher::Publish() {
    return PublishImage() && PublishInfo();
  }

  bool CarlaISCameraPublisher::PublishImage() {
    return _impl->_dds_image->Write(&_impl->_image);
  }

  bool CarlaISCameraPublisher::PublishInfo() {
    return _impl->_dds_info->Write(&_impl->_info);
  }

  void CarlaISCameraPublisher::SetImageData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, const uint8_t* data) {
    std::vector<uint8_t> vector_data;
    const size_t size = height * width * 4;
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);
    SetData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaISCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs_msg_RegionOfInterest roi;
    roi.x_offset = x_offset;
    roi.y_offset = y_offset;
    roi.height = height;
    roi.width = width;
    roi.do_rectify = do_rectify;
    _impl->_info.roi = roi;
  }

  void CarlaISCameraPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = _frame_id;

    _impl->_image.header = header;
    _impl->_image.width = width;
    _impl->_image.height = height;
    _impl->_image.encoding = "bgra8";
    _impl->_image.is_bigendian = 0;
    _impl->_image.step = _impl->_image.width * sizeof(uint8_t) * 4;
    _impl->_image.data = data;
  }

  void CarlaISCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = _frame_id;
    _impl->_info.header = header;
  }

  CarlaISCameraPublisher::CarlaISCameraPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaISCameraPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaISCameraPublisher::~CarlaISCameraPublisher() = default;
  CarlaISCameraPublisher::CarlaISCameraPublisher(const CarlaISCameraPublisher&) = default;
  CarlaISCameraPublisher& CarlaISCameraPublisher::operator=(const CarlaISCameraPublisher&) = default;
  CarlaISCameraPublisher::CarlaISCameraPublisher(CarlaISCameraPublisher&&) = default;
  CarlaISCameraPublisher& CarlaISCameraPublisher::operator=(CarlaISCameraPublisher&&) = default;
}
}
