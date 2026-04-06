#include "CarlaSSCameraPublisher.h"

#include <string>
#include <cstring>
#include <vector>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/dds/cyclonedds/CycloneDDSTopicHelper.h"
#include "carla/ros2/dds/cyclonedds/conversions.hpp"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "Header.h"
#include "Time.h"
#include "CycloneCameraInfoHelper.h"

namespace carla {
namespace ros2 {

  struct CarlaSSCameraPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_Image _image {};
    std::string _frame_id_store;
    std::string _encoding_store;
    std::vector<uint8_t> _data_store;
    TopicConfig _config {};
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

  bool CarlaSSCameraPublisher::HasBeenInitialized() const {
    return _impl_info->_init;
  }

  void CarlaSSCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    cyclone_helpers::InitCameraInfo(_impl_info->_info, height, width, fov, _impl_info->_d_store, _impl_info->_distortion_model_store);
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _impl_info->_init = true;
  }

  bool CarlaSSCameraPublisher::Init() {
    TopicConfig config;
    config.domain_id = GetDomainId();
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;
    return Init(config);
  }

  bool CarlaSSCameraPublisher::Init(const TopicConfig& config) {
    _impl->_config = config;
    return InitImage() && InitInfo();
  }

  bool CarlaSSCameraPublisher::InitImage() {
    _impl->_participant = dds_create_participant(_impl->_config.domain_id, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string publisher_type { _impl->_config.suffix.empty() ? "/image" : _impl->_config.suffix };
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;

    if (const auto custom_topic_name = ValidTopicName(publisher_type)) {
        topic_name = custom_topic_name.value();
    }
    topic_name = SanitizeTopicName(topic_name);
    _impl->_topic = dds_create_topic(_impl->_participant, &sensor_msgs_msg_Image_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    dds_qos_t* qos = dds_create_qos();
    configure_cyclone_qos(_impl->_config, qos);
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

  bool CarlaSSCameraPublisher::InitInfo() {
    _impl_info->_participant = dds_create_participant(_impl->_config.domain_id, nullptr, nullptr);
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

    if (const auto custom_topic_name = ValidTopicName(publisher_type)) {
        topic_name = custom_topic_name.value();
    }
    topic_name = SanitizeTopicName(topic_name);
    _impl_info->_topic = dds_create_topic(_impl_info->_participant, &sensor_msgs_msg_CameraInfo_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl_info->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_impl_info->_participant);
        return false;
    }

    dds_qos_t* qos_info = dds_create_qos();
    configure_cyclone_qos(_impl->_config, qos_info);
    _impl_info->_writer = dds_create_writer(_impl_info->_participant, _impl_info->_topic, qos_info, nullptr);
    dds_delete_qos(qos_info);
    if (_impl_info->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_impl_info->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool CarlaSSCameraPublisher::Publish() {
    return PublishImage() && PublishInfo();
  }

  bool CarlaSSCameraPublisher::PublishImage() {
    return dds_write(_impl->_writer, &_impl->_image) >= 0;
  }

  bool CarlaSSCameraPublisher::PublishInfo() {
    return dds_write(_impl_info->_writer, &_impl_info->_info) >= 0;
  }

  void CarlaSSCameraPublisher::SetImageData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, const uint8_t* data) {
    std::vector<uint8_t> vector_data;
    const size_t size = height * width * 4;
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);
    SetData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaSSCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs_msg_RegionOfInterest roi;
    roi.x_offset = x_offset;
    roi.y_offset = y_offset;
    roi.height = height;
    roi.width = width;
    roi.do_rectify = do_rectify;
    _impl_info->_info.roi = roi;
  }

  void CarlaSSCameraPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
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

  void CarlaSSCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl_info->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl_info->_frame_id_store.c_str());
    _impl_info->_info.header = header;
  }

  CarlaSSCameraPublisher::CarlaSSCameraPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaSSCameraPublisherImpl>()),
  _impl_info(std::make_shared<CarlaCameraInfoPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaSSCameraPublisher::~CarlaSSCameraPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
    if (_impl_info && _impl_info->_participant > 0)
        dds_delete(_impl_info->_participant);
  }

  CarlaSSCameraPublisher::CarlaSSCameraPublisher(const CarlaSSCameraPublisher&) = default;
  CarlaSSCameraPublisher& CarlaSSCameraPublisher::operator=(const CarlaSSCameraPublisher&) = default;
  CarlaSSCameraPublisher::CarlaSSCameraPublisher(CarlaSSCameraPublisher&&) = default;
  CarlaSSCameraPublisher& CarlaSSCameraPublisher::operator=(CarlaSSCameraPublisher&&) = default;
}
}
