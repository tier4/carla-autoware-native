#include "CarlaOpticalFlowCameraPublisher.h"

#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "dds/dds.h"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "Header.h"
#include "Time.h"
#include "CycloneCameraInfoHelper.h"

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

namespace carla {
namespace ros2 {

  struct CarlaOpticalFlowCameraPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_Image _image {};
    std::string _frame_id_store;
    std::string _encoding_store;
    std::vector<uint8_t> _data_store;
  };

  struct CarlaCameraInfoPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    bool _init {false};
    sensor_msgs_msg_CameraInfo _info {};
    std::string _frame_id_store;
    std::vector<double> _d_store;
    std::string _distortion_model_store;
  };

  bool CarlaOpticalFlowCameraPublisher::HasBeenInitialized() const {
    return _impl_info->_init;
  }

  void CarlaOpticalFlowCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    cyclone_helpers::InitCameraInfo(_impl_info->_info, height, width, fov, _impl_info->_d_store, _impl_info->_distortion_model_store);
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _impl_info->_init = true;
  }

  bool CarlaOpticalFlowCameraPublisher::Init() {
    return InitImage() && InitInfo();
  }

  bool CarlaOpticalFlowCameraPublisher::InitImage() {
    _impl->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string publisher_type {"/image"};
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;

    _impl->_topic = dds_create_topic(_impl->_participant, &sensor_msgs_msg_Image_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaOpticalFlowCameraPublisher::InitInfo() {
    _impl_info->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl_info->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string publisher_type {"/camera_info"};
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;

    _impl_info->_topic = dds_create_topic(_impl_info->_participant, &sensor_msgs_msg_CameraInfo_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl_info->_topic < 0) {
        std::cerr << "Failed to create Topic" << std::endl;
        dds_delete(_impl_info->_participant);
        return false;
    }

    _impl_info->_writer = dds_create_writer(_impl_info->_participant, _impl_info->_topic, nullptr, nullptr);
    if (_impl_info->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_impl_info->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool CarlaOpticalFlowCameraPublisher::Publish() {
    return PublishImage() && PublishInfo();
  }

  bool CarlaOpticalFlowCameraPublisher::PublishImage() {
    return dds_write(_impl->_writer, &_impl->_image) >= 0;
  }

  bool CarlaOpticalFlowCameraPublisher::PublishInfo() {
    return dds_write(_impl_info->_writer, &_impl_info->_info) >= 0;
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
        if (angle < 0) { angle = 360.0f + angle; }
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

        float r = 0, g = 0, b = 0;
        const unsigned int angle_case = static_cast<const unsigned int>(H_60);
        switch (angle_case) {
        case 0: r = C; g = X; b = 0; break;
        case 1: r = X; g = C; b = 0; break;
        case 2: r = 0; g = C; b = X; break;
        case 3: r = 0; g = X; b = C; break;
        case 4: r = X; g = 0; b = C; break;
        case 5: r = C; g = 0; b = X; break;
        default: r = 1; g = 1; b = 1; break;
        }

        vector_data[data_index++] = static_cast<uint8_t>((b + m) * 255.0f);
        vector_data[data_index++] = static_cast<uint8_t>((g + m) * 255.0f);
        vector_data[data_index++] = static_cast<uint8_t>((r + m) * 255.0f);
        vector_data[data_index++] = 0;
    }
    SetData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaOpticalFlowCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs_msg_RegionOfInterest roi;
    roi.x_offset = x_offset;
    roi.y_offset = y_offset;
    roi.height = height;
    roi.width = width;
    roi.do_rectify = do_rectify;
    _impl_info->_info.roi = roi;
  }

  void CarlaOpticalFlowCameraPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    _impl->_image.header = header;
    _impl->_image.width = width;
    _impl->_image.height = height;
    _impl->_encoding_store = "bgra8";
    _impl->_image.encoding = const_cast<char*>(_impl->_encoding_store.c_str());
    _impl->_image.is_bigendian = 0;
    _impl->_image.step = _impl->_image.width * sizeof(uint8_t) * 4;
    _impl->_data_store = std::move(data);
    _impl->_image.data._buffer = _impl->_data_store.data();
    _impl->_image.data._length = static_cast<uint32_t>(_impl->_data_store.size());
    _impl->_image.data._maximum = static_cast<uint32_t>(_impl->_data_store.size());
    _impl->_image.data._release = false;
  }

  void CarlaOpticalFlowCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl_info->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl_info->_frame_id_store.c_str());
    _impl_info->_info.header = header;
  }

  CarlaOpticalFlowCameraPublisher::CarlaOpticalFlowCameraPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaOpticalFlowCameraPublisherImpl>()),
  _impl_info(std::make_shared<CarlaCameraInfoPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaOpticalFlowCameraPublisher::~CarlaOpticalFlowCameraPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
    if (_impl_info && _impl_info->_participant > 0)
        dds_delete(_impl_info->_participant);
  }

  CarlaOpticalFlowCameraPublisher::CarlaOpticalFlowCameraPublisher(const CarlaOpticalFlowCameraPublisher&) = default;
  CarlaOpticalFlowCameraPublisher& CarlaOpticalFlowCameraPublisher::operator=(const CarlaOpticalFlowCameraPublisher&) = default;
  CarlaOpticalFlowCameraPublisher::CarlaOpticalFlowCameraPublisher(CarlaOpticalFlowCameraPublisher&&) = default;
  CarlaOpticalFlowCameraPublisher& CarlaOpticalFlowCameraPublisher::operator=(CarlaOpticalFlowCameraPublisher&&) = default;
}
}
