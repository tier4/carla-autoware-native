#include "CarlaDVSCameraPublisher.h"

#include <string>
#include <cstring>
#include <vector>
#include <iostream>

#include "carla/sensor/data/DVSEvent.h"

#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "Header.h"
#include "Time.h"
#include "CycloneCameraInfoHelper.h"

namespace carla {
namespace ros2 {

  struct CarlaDVSCameraPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_Image _image {};
    std::string _frame_id_store;
    std::string _encoding_store;
    std::vector<uint8_t> _image_data_store;
  };

  struct CarlaCameraInfoPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    bool _init {false};
    sensor_msgs_msg_CameraInfo _ci {};
    std::string _frame_id_store;
    std::vector<double> _d_store;
    std::string _distortion_model_store;
  };

  struct CarlaPointCloudPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_PointCloud2 _pc {};
    std::vector<uint8_t> _pc_data_store;
    std::vector<sensor_msgs_msg_PointField> _fields_store;
  };

  bool CarlaDVSCameraPublisher::HasBeenInitialized() const {
    return _info->_init;
  }

  void CarlaDVSCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    cyclone_helpers::InitCameraInfo(_info->_ci, height, width, fov, _info->_d_store, _info->_distortion_model_store);
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _info->_init = true;
  }

  bool CarlaDVSCameraPublisher::Init() {
    return InitImage() && InitInfo() && InitPointCloud();
  }

  bool CarlaDVSCameraPublisher::InitImage() {
    _impl->_participant = dds_create_participant(GetDomainId(), nullptr, nullptr);
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

    _impl->_writer = dds_create_writer(_impl->_participant, _impl->_topic, nullptr, nullptr);
    if (_impl->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool CarlaDVSCameraPublisher::InitInfo() {
    _info->_participant = dds_create_participant(GetDomainId(), nullptr, nullptr);
    if (_info->_participant < 0) {
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
    _info->_topic = dds_create_topic(_info->_participant, &sensor_msgs_msg_CameraInfo_desc, topic_name.c_str(), nullptr, nullptr);
    if (_info->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_info->_participant);
        return false;
    }

    _info->_writer = dds_create_writer(_info->_participant, _info->_topic, nullptr, nullptr);
    if (_info->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_info->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool CarlaDVSCameraPublisher::InitPointCloud() {
    _point_cloud->_participant = dds_create_participant(GetDomainId(), nullptr, nullptr);
    if (_point_cloud->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string publisher_type {"/point_cloud"};
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
    _point_cloud->_topic = dds_create_topic(_point_cloud->_participant, &sensor_msgs_msg_PointCloud2_desc, topic_name.c_str(), nullptr, nullptr);
    if (_point_cloud->_topic < 0) {
        std::cerr << "CycloneDDS: Failed to create Topic in " << type() << ": " << dds_strretcode(-_impl->_topic) << std::endl;
        dds_delete(_point_cloud->_participant);
        return false;
    }

    _point_cloud->_writer = dds_create_writer(_point_cloud->_participant, _point_cloud->_topic, nullptr, nullptr);
    if (_point_cloud->_writer < 0) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        dds_delete(_point_cloud->_participant);
        return false;
    }

    _frame_id = _name;
    return true;
  }

  bool CarlaDVSCameraPublisher::Publish() {
    return PublishImage() && PublishInfo() && PublishPointCloud();
  }

  bool CarlaDVSCameraPublisher::PublishImage() {
    return dds_write(_impl->_writer, &_impl->_image) >= 0;
  }

  bool CarlaDVSCameraPublisher::PublishInfo() {
    return dds_write(_info->_writer, &_info->_ci) >= 0;
  }

  bool CarlaDVSCameraPublisher::PublishPointCloud() {
    return dds_write(_point_cloud->_writer, &_point_cloud->_pc) >= 0;
  }

  void CarlaDVSCameraPublisher::SetImageData(int32_t seconds, uint32_t nanoseconds, size_t elements, size_t height, size_t width, const uint8_t* data) {
    std::vector<uint8_t> im_data;
    const size_t im_size = width * height * 3;
    im_data.resize(im_size);
    carla::sensor::data::DVSEvent* vec_event = (carla::sensor::data::DVSEvent*)&data[0];
    for (size_t i = 0; i < elements; ++i, ++vec_event) {
        size_t index = (vec_event->y * width + vec_event->x) * 3 + (static_cast<int>(vec_event->pol) * 2);
        im_data[index] = 255;
    }

    SetData(seconds, nanoseconds, height, width, std::move(im_data));
  }

  void CarlaDVSCameraPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());
    _impl->_image.header = header;
    _info->_ci.header = header;
    _point_cloud->_pc.header = header;

    _impl->_image.width = width;
    _impl->_image.height = height;
    _impl->_encoding_store = "bgr8";
    _impl->_image.encoding = const_cast<char*>(_impl->_encoding_store.c_str());
    _impl->_image.is_bigendian = 0;
    _impl->_image.step = _impl->_image.width * sizeof(uint8_t) * 3;
    _impl->_image_data_store = std::move(data);
    _impl->_image.data._buffer = _impl->_image_data_store.data();
    _impl->_image.data._length = static_cast<uint32_t>(_impl->_image_data_store.size());
    _impl->_image.data._maximum = static_cast<uint32_t>(_impl->_image_data_store.size());
    _impl->_image.data._release = false;
  }

  void CarlaDVSCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _info->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_info->_frame_id_store.c_str());
    _info->_ci.header = header;
  }

  void CarlaDVSCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs_msg_RegionOfInterest roi;
    roi.x_offset = x_offset;
    roi.y_offset = y_offset;
    roi.height = height;
    roi.width = width;
    roi.do_rectify = do_rectify;
    _info->_ci.roi = roi;
  }

  void CarlaDVSCameraPublisher::SetPointCloudData(size_t height, size_t width, size_t elements, const uint8_t* data) {

    std::vector<uint8_t> vector_data;
    const size_t size = height * width;
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);

    sensor_msgs_msg_PointField descriptor1;
    descriptor1.name = const_cast<char*>("x");
    descriptor1.offset = 0;
    descriptor1.datatype = 4;
    descriptor1.count = 1;
    sensor_msgs_msg_PointField descriptor2;
    descriptor2.name = const_cast<char*>("y");
    descriptor2.offset = 2;
    descriptor2.datatype = 4;
    descriptor2.count = 1;
    sensor_msgs_msg_PointField descriptor3;
    descriptor3.name = const_cast<char*>("t");
    descriptor3.offset = 4;
    descriptor3.datatype = 8;
    descriptor3.count = 1;
    sensor_msgs_msg_PointField descriptor4;
    descriptor4.name = const_cast<char*>("pol");
    descriptor4.offset = 12;
    descriptor4.datatype = 1;
    descriptor4.count = 1;

    const size_t point_size = sizeof(carla::sensor::data::DVSEvent);
    _point_cloud->_pc.width = width;
    _point_cloud->_pc.height = height;
    _point_cloud->_pc.is_bigendian = false;
    _point_cloud->_fields_store = {descriptor1, descriptor2, descriptor3, descriptor4};
    _point_cloud->_pc.fields._buffer = _point_cloud->_fields_store.data();
    _point_cloud->_pc.fields._length = static_cast<uint32_t>(_point_cloud->_fields_store.size());
    _point_cloud->_pc.fields._maximum = static_cast<uint32_t>(_point_cloud->_fields_store.size());
    _point_cloud->_pc.fields._release = false;
    _point_cloud->_pc.point_step = point_size;
    _point_cloud->_pc.row_step = width * point_size;
    _point_cloud->_pc.is_dense = false;
    _point_cloud->_pc_data_store = std::move(vector_data);
    _point_cloud->_pc.data._buffer = _point_cloud->_pc_data_store.data();
    _point_cloud->_pc.data._length = static_cast<uint32_t>(_point_cloud->_pc_data_store.size());
    _point_cloud->_pc.data._maximum = static_cast<uint32_t>(_point_cloud->_pc_data_store.size());
    _point_cloud->_pc.data._release = false;
  }

  CarlaDVSCameraPublisher::CarlaDVSCameraPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaDVSCameraPublisherImpl>()),
  _info(std::make_shared<CarlaCameraInfoPublisherImpl>()),
  _point_cloud(std::make_shared<CarlaPointCloudPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaDVSCameraPublisher::~CarlaDVSCameraPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
    if (_info && _info->_participant > 0)
        dds_delete(_info->_participant);
    if (_point_cloud && _point_cloud->_participant > 0)
        dds_delete(_point_cloud->_participant);
  }

  CarlaDVSCameraPublisher::CarlaDVSCameraPublisher(const CarlaDVSCameraPublisher&) = default;
  CarlaDVSCameraPublisher& CarlaDVSCameraPublisher::operator=(const CarlaDVSCameraPublisher&) = default;
  CarlaDVSCameraPublisher::CarlaDVSCameraPublisher(CarlaDVSCameraPublisher&&) = default;
  CarlaDVSCameraPublisher& CarlaDVSCameraPublisher::operator=(CarlaDVSCameraPublisher&&) = default;
}
}
