#include "CarlaDVSCameraPublisher.h"

#include <string>

#include "carla/sensor/data/DVSEvent.h"

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Image.h"
#include "CameraInfo.h"
#include "RegionOfInterest.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "Header.h"
#include "Time.h"

namespace carla {
namespace ros2 {

  struct CarlaDVSCameraPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds_image;
    std::unique_ptr<DDSPublisherImpl> _dds_info;
    std::unique_ptr<DDSPublisherImpl> _dds_pointcloud;
    sensor_msgs::msg::Image _image {};
    sensor_msgs::msg::CameraInfo _ci {};
    sensor_msgs::msg::PointCloud2 _pc {};
    bool _info_init {false};
  };

  bool CarlaDVSCameraPublisher::HasBeenInitialized() const {
    return _impl->_info_init;
  }

  void CarlaDVSCameraPublisher::InitInfoData(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, float fov, bool do_rectify) {
    _impl->_ci = std::move(sensor_msgs::msg::CameraInfo(height, width, fov));
    SetInfoRegionOfInterest(x_offset, y_offset, height, width, do_rectify);
    _impl->_info_init = true;
  }

  bool CarlaDVSCameraPublisher::Init(const TopicConfig& config) {
    return InitImage(config) && InitInfo(config) && InitPointCloud(config);
  }

  bool CarlaDVSCameraPublisher::InitImage(const TopicConfig& config) {
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

  bool CarlaDVSCameraPublisher::InitInfo(const TopicConfig& config) {
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

  bool CarlaDVSCameraPublisher::InitPointCloud(const TopicConfig& config) {
    _impl->_dds_pointcloud = CreateDDSPublisher("sensor_msgs::msg::PointCloud2");
    if (!_impl->_dds_pointcloud) return false;

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

    if (!_impl->_dds_pointcloud->Init(config, _name, topic_name, /*use_preallocated_realloc=*/true)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaDVSCameraPublisher::Publish() {
    return PublishImage() && PublishInfo() && PublishPointCloud();
  }

  bool CarlaDVSCameraPublisher::PublishImage() {
    return _impl->_dds_image->Write(&_impl->_image);
  }

  bool CarlaDVSCameraPublisher::PublishInfo() {
    return _impl->_dds_info->Write(&_impl->_ci);
  }

  bool CarlaDVSCameraPublisher::PublishPointCloud() {
    return _impl->_dds_pointcloud->Write(&_impl->_pc);
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
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);
    _impl->_image.header(header);
    _impl->_ci.header(header);
    _impl->_pc.header(header);

    _impl->_image.width(width);
    _impl->_image.height(height);
    _impl->_image.encoding("bgr8");
    _impl->_image.is_bigendian(0);
    _impl->_image.step(_impl->_image.width() * sizeof(uint8_t) * 3);
    _impl->_image.data(std::move(data));
  }

  void CarlaDVSCameraPublisher::SetCameraInfoData(int32_t seconds, uint32_t nanoseconds) {
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);
  }

  void CarlaDVSCameraPublisher::SetInfoRegionOfInterest(uint32_t x_offset, uint32_t y_offset, uint32_t height, uint32_t width, bool do_rectify) {
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset(x_offset);
    roi.y_offset(y_offset);
    roi.height(height);
    roi.width(width);
    roi.do_rectify(do_rectify);
    _impl->_ci.roi(roi);
  }

  void CarlaDVSCameraPublisher::SetPointCloudData(size_t height, size_t width, size_t elements, const uint8_t* data) {

    std::vector<uint8_t> vector_data;
    const size_t size = height * width;
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);

    sensor_msgs::msg::PointField descriptor1;
    descriptor1.name("x");
    descriptor1.offset(0);
    descriptor1.datatype(sensor_msgs::msg::PointField__UINT16);
    descriptor1.count(1);
    sensor_msgs::msg::PointField descriptor2;
    descriptor2.name("y");
    descriptor2.offset(2);
    descriptor2.datatype(sensor_msgs::msg::PointField__UINT16);
    descriptor2.count(1);
    sensor_msgs::msg::PointField descriptor3;
    descriptor3.name("t");
    descriptor3.offset(4);
    descriptor3.datatype(sensor_msgs::msg::PointField__FLOAT64);
    descriptor3.count(1);
    sensor_msgs::msg::PointField descriptor4;
    descriptor4.name("pol");
    descriptor4.offset(12);
    descriptor4.datatype(sensor_msgs::msg::PointField__INT8);
    descriptor4.count(1);

    const size_t point_size = sizeof(carla::sensor::data::DVSEvent);
    _impl->_pc.width(width);
    _impl->_pc.height(height);
    _impl->_pc.is_bigendian(false);
    _impl->_pc.fields({descriptor1, descriptor2, descriptor3, descriptor4});
    _impl->_pc.point_step(point_size);
    _impl->_pc.row_step(width * point_size);
    _impl->_pc.is_dense(false);
    _impl->_pc.data(std::move(vector_data));
  }

  CarlaDVSCameraPublisher::CarlaDVSCameraPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaDVSCameraPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaDVSCameraPublisher::~CarlaDVSCameraPublisher() = default;
  CarlaDVSCameraPublisher::CarlaDVSCameraPublisher(const CarlaDVSCameraPublisher&) = default;
  CarlaDVSCameraPublisher& CarlaDVSCameraPublisher::operator=(const CarlaDVSCameraPublisher&) = default;
  CarlaDVSCameraPublisher::CarlaDVSCameraPublisher(CarlaDVSCameraPublisher&&) = default;
  CarlaDVSCameraPublisher& CarlaDVSCameraPublisher::operator=(CarlaDVSCameraPublisher&&) = default;
}
}
