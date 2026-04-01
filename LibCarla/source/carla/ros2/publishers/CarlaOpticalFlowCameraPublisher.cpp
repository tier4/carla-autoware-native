#include "CarlaOpticalFlowCameraPublisher.h"

#include <string>
#include <cmath>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "Header.h"
#include "Time.h"

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

namespace carla {
namespace ros2 {

  struct CarlaOpticalFlowCameraPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds_image;
    std::unique_ptr<DDSPublisherImpl> _dds_info;
    sensor_msgs::msg::Image _image {};
    sensor_msgs::msg::CameraInfo _info {};
    bool _info_init {false};
  };

  bool CarlaOpticalFlowCameraPublisher::HasBeenInitialized() const {
    return _impl->_info_init;
  }

  void CarlaOpticalFlowCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    _impl->_info = std::move(sensor_msgs::msg::CameraInfo(height, width, fov));
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _impl->_info_init = true;
  }

  bool CarlaOpticalFlowCameraPublisher::Init(const TopicConfig& config) {
    return InitImage(config) && InitInfo(config);
  }

  bool CarlaOpticalFlowCameraPublisher::InitImage(const TopicConfig& config) {
    _impl->_dds_image = CreateDDSPublisher("sensor_msgs::msg::Image");
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

  bool CarlaOpticalFlowCameraPublisher::InitInfo(const TopicConfig& config) {
    _impl->_dds_info = CreateDDSPublisher("sensor_msgs::msg::CameraInfo");
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

  bool CarlaOpticalFlowCameraPublisher::Publish() {
    return PublishImage() && PublishInfo();
  }

  bool CarlaOpticalFlowCameraPublisher::PublishImage() {
    return _impl->_dds_image->Write(&_impl->_image);
  }

  bool CarlaOpticalFlowCameraPublisher::PublishInfo() {
    return _impl->_dds_info->Write(&_impl->_info);
  }

  void CarlaOpticalFlowCameraPublisher::SetImageData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, const float* data) {
    constexpr float pi = 3.1415f;
    constexpr float rad2ang = 360.0f/(2.0f*pi);
    const size_t max_index = width * height * 2;
    std::vector<uint8_t> vector_data;
    vector_data.resize(height * width * 4);
    size_t data_index = 0;
    for (size_t index = 0; index < max_index; index += 2) {
        const float vx = data[index];
        const float vy = data[index + 1];
        float angle = 180.0f + std::atan2(vy, vx) * rad2ang;
        if (angle < 0)
        {
            angle = 360.0f + angle;
        }
        angle = std::fmod(angle, 360.0f);

        const float norm = std::sqrt(vx * vx + vy * vy);
        const float shift = 0.999f;
        const float a = 1.0f / std::log(0.1f + shift);
        const float intensity = CLAMP<float>(a * std::log(norm + shift), 0.0f, 1.0f);

        const float& H = angle;
        const float S = 1.0f;
        const float V = intensity;
        const float H_60 = H * (1.0f / 60.0f);

        const float C = V * S;
        const float X = C * (1.0f - std::abs(std::fmod(H_60, 2.0f) - 1.0f));
        const float m = V - C;

        float r = 0;
        float g = 0;
        float b = 0;
        const unsigned int angle_case = static_cast<const unsigned int>(H_60);
        switch (angle_case) {
        case 0:
            r = C;
            g = X;
            b = 0;
            break;
        case 1:
            r = X;
            g = C;
            b = 0;
            break;
        case 2:
            r = 0;
            g = C;
            b = X;
            break;
        case 3:
            r = 0;
            g = X;
            b = C;
            break;
        case 4:
            r = X;
            g = 0;
            b = C;
            break;
        case 5:
            r = C;
            g = 0;
            b = X;
            break;
        default:
            r = 1;
            g = 1;
            b = 1;
            break;
        }

        const uint8_t R = static_cast<uint8_t>((r + m) * 255.0f);
        const uint8_t G = static_cast<uint8_t>((g + m) * 255.0f);
        const uint8_t B = static_cast<uint8_t>((b + m) * 255.0f);

        vector_data[data_index++] = B;
        vector_data[data_index++] = G;
        vector_data[data_index++] = R;
        vector_data[data_index++] = 0;
    }
    SetData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaOpticalFlowCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset(x_offset);
    roi.y_offset(y_offset);
    roi.height(height);
    roi.width(width);
    roi.do_rectify(do_rectify);
    _impl->_info.roi(roi);
  }

  void CarlaOpticalFlowCameraPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);

    _impl->_image.header(std::move(header));
    _impl->_image.width(width);
    _impl->_image.height(height);
    _impl->_image.encoding("bgra8");
    _impl->_image.is_bigendian(0);
    _impl->_image.step(_impl->_image.width() * sizeof(uint8_t) * 4);
    _impl->_image.data(std::move(data));
  }

  void CarlaOpticalFlowCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);
    _impl->_info.header(header);
  }

  CarlaOpticalFlowCameraPublisher::CarlaOpticalFlowCameraPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaOpticalFlowCameraPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaOpticalFlowCameraPublisher::~CarlaOpticalFlowCameraPublisher() = default;
  CarlaOpticalFlowCameraPublisher::CarlaOpticalFlowCameraPublisher(const CarlaOpticalFlowCameraPublisher&) = default;
  CarlaOpticalFlowCameraPublisher& CarlaOpticalFlowCameraPublisher::operator=(const CarlaOpticalFlowCameraPublisher&) = default;
  CarlaOpticalFlowCameraPublisher::CarlaOpticalFlowCameraPublisher(CarlaOpticalFlowCameraPublisher&&) = default;
  CarlaOpticalFlowCameraPublisher& CarlaOpticalFlowCameraPublisher::operator=(CarlaOpticalFlowCameraPublisher&&) = default;
}
}
