#include "CarlaLidarPublisher.h"

#include <string>
#include <cstring>
#include <sstream>
#include <cmath>

#include "carla/ros2/dds/fastdds/conversions.hpp"

#include "carla/ros2/dds/fastdds/types/PointCloud2PubSubTypes.h"
#include "carla/ros2/dds/fastdds/listeners/CarlaListener.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/qos/QosPolicies.h>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

namespace carla {
namespace ros2 {

  namespace efd = eprosima::fastdds::dds;
  using erc = eprosima::fastrtps::types::ReturnCode_t;

  struct CarlaLidarPublisherImpl {
    efd::DomainParticipant* _participant { nullptr };
    efd::Publisher* _publisher { nullptr };
    efd::Topic* _topic { nullptr };
    efd::DataWriter* _datawriter { nullptr };
    efd::TypeSupport _type { new sensor_msgs::msg::PointCloud2PubSubType() };
    CarlaListener _listener {};
    sensor_msgs::msg::PointCloud2 _lidar {};
  };

  bool CarlaLidarPublisher::Init() {
    TopicConfig config;
    config.domain_id = 0;
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;
    return Init(config);
  }

  bool CarlaLidarPublisher::Init(const TopicConfig& config) {
    if (_impl->_type == nullptr) {
        std::cerr << "Invalid TypeSupport" << std::endl;
        return false;
    }

    efd::DomainParticipantQos pqos = efd::PARTICIPANT_QOS_DEFAULT;
    pqos.name(_name);
    auto factory = efd::DomainParticipantFactory::get_instance();
    _impl->_participant = factory->create_participant(config.domain_id, pqos);
    if (_impl->_participant == nullptr) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }
    _impl->_type.register_type(_impl->_participant);

    efd::PublisherQos pubqos = efd::PUBLISHER_QOS_DEFAULT;
    _impl->_publisher = _impl->_participant->create_publisher(pubqos, nullptr);
    if (_impl->_publisher == nullptr) {
      std::cerr << "Failed to create Publisher" << std::endl;
      return false;
    }

    efd::TopicQos tqos = efd::TOPIC_QOS_DEFAULT;
    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    _impl->_topic = _impl->_participant->create_topic(topic_name, _impl->_type->getName(), tqos);
    if (_impl->_topic == nullptr) {
        std::cerr << "Failed to create Topic" << std::endl;
        return false;
    }

    efd::DataWriterQos wqos = efd::DATAWRITER_QOS_DEFAULT;
    configure_qos(config, wqos);
    wqos.endpoint().history_memory_policy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    efd::DataWriterListener* listener = (efd::DataWriterListener*)_impl->_listener._impl.get();
    _impl->_datawriter = _impl->_publisher->create_datawriter(_impl->_topic, wqos, listener);
    if (_impl->_datawriter == nullptr) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaLidarPublisher::Publish() {
    eprosima::fastrtps::rtps::InstanceHandle_t instance_handle;
    eprosima::fastrtps::types::ReturnCode_t rcode = _impl->_datawriter->write(&_impl->_lidar, instance_handle);
    if (rcode == erc::ReturnCodeValue::RETCODE_OK) {
        return true;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ERROR) {
        std::cerr << "RETCODE_ERROR" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_UNSUPPORTED) {
        std::cerr << "RETCODE_UNSUPPORTED" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_BAD_PARAMETER) {
        std::cerr << "RETCODE_BAD_PARAMETER" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_PRECONDITION_NOT_MET) {
        std::cerr << "RETCODE_PRECONDITION_NOT_MET" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_OUT_OF_RESOURCES) {
        std::cerr << "RETCODE_OUT_OF_RESOURCES" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NOT_ENABLED) {
        std::cerr << "RETCODE_NOT_ENABLED" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_IMMUTABLE_POLICY) {
        std::cerr << "RETCODE_IMMUTABLE_POLICY" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_INCONSISTENT_POLICY) {
        std::cerr << "RETCODE_INCONSISTENT_POLICY" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ALREADY_DELETED) {
        std::cerr << "RETCODE_ALREADY_DELETED" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_TIMEOUT) {
        std::cerr << "RETCODE_TIMEOUT" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NO_DATA) {
        std::cerr << "RETCODE_NO_DATA" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ILLEGAL_OPERATION) {
        std::cerr << "RETCODE_ILLEGAL_OPERATION" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NOT_ALLOWED_BY_SECURITY) {
        std::cerr << "RETCODE_NOT_ALLOWED_BY_SECURITY" << std::endl;
        return false;
    }
    std::cerr << "UNKNOWN" << std::endl;
    return false;
  }


  void CarlaLidarPublisher::ConvertToRosFormat(const size_t height, const size_t width, float * data) {
    float const * const end = data + height * width;
    // Invert Y axis
    for (auto it = data + 1; it < end; it += 4) {
      *it *= -1.0f;
    }
  }

void CarlaLidarPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, float* data) {
    ConvertToRosFormat(height, width, data);

    std::vector<uint8_t> vector_data;
    const size_t size = height * width * sizeof(float);
    vector_data.resize(size);
    std::memcpy(&vector_data[0], &data[0], size);
    SetData(seconds, nanoseconds, height, width, std::move(vector_data));
  }

  void CarlaLidarPublisher::SetData(int32_t seconds, uint32_t nanoseconds, size_t height, size_t width, std::vector<uint8_t>&& data) {
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
    descriptor4.name("intensity");
    descriptor4.offset(12);
    descriptor4.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor4.count(1);

    const size_t point_size = 4 * sizeof(float);
    _impl->_lidar.header(std::move(header));
    _impl->_lidar.width(width / 4);
    _impl->_lidar.height(height);
    _impl->_lidar.is_bigendian(false);
    _impl->_lidar.fields({descriptor1, descriptor2, descriptor3, descriptor4});
    _impl->_lidar.point_step(point_size);
    _impl->_lidar.row_step(width * sizeof(float));
    _impl->_lidar.is_dense(false); //True if there are not invalid points
    _impl->_lidar.data(std::move(data));
  }

  void CarlaLidarPublisher::SetDataEx(const int32_t seconds, const uint32_t nanoseconds,const size_t height, const size_t width,
                                      float * data, const size_t header_size, uint32_t * header_data, const std::vector<float> & vertical_angles) {
    ConvertToRosFormat(height, width, data);

    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);

    uint32_t offset{0U};

    sensor_msgs::msg::PointField descriptor1;
    descriptor1.name("x");
    descriptor1.offset(offset);
    descriptor1.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor1.count(1);
    offset += sizeof(float);
    sensor_msgs::msg::PointField descriptor2;
    descriptor2.name("y");
    descriptor2.offset(offset);
    descriptor2.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor2.count(1);
    offset += sizeof(float);
    sensor_msgs::msg::PointField descriptor3;
    descriptor3.name("z");
    descriptor3.offset(offset);
    descriptor3.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor3.count(1);
    offset += sizeof(float);
    sensor_msgs::msg::PointField descriptor4;
    descriptor4.name("intensity");
    descriptor4.offset(offset);
    descriptor4.datatype(sensor_msgs::msg::PointField__UINT8);
    descriptor4.count(1);
    offset += sizeof(uint8_t);
    // Dummy data
    sensor_msgs::msg::PointField descriptor5;
    descriptor5.name("return_type");
    descriptor5.offset(offset);
    descriptor5.datatype(sensor_msgs::msg::PointField__UINT8);
    descriptor5.count(1);
    offset += sizeof(uint8_t);
    sensor_msgs::msg::PointField descriptor6;
    descriptor6.name("channel");
    descriptor6.offset(offset);
    descriptor6.datatype(sensor_msgs::msg::PointField__UINT16);
    descriptor6.count(1);
    offset += sizeof(uint16_t);
    sensor_msgs::msg::PointField descriptor7;
    descriptor7.name("azimuth");
    descriptor7.offset(offset);
    descriptor7.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor7.count(1);
    offset += sizeof(float);
    sensor_msgs::msg::PointField descriptor8;
    descriptor8.name("elevation");
    descriptor8.offset(offset);
    descriptor8.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor8.count(1);
    offset += sizeof(float);
    sensor_msgs::msg::PointField descriptor9;
    descriptor9.name("distance");
    descriptor9.offset(offset);
    descriptor9.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor9.count(1);
    offset += sizeof(float);
    sensor_msgs::msg::PointField descriptor10;
    descriptor10.name("time_stamp");
    descriptor10.offset(offset);
    descriptor10.datatype(sensor_msgs::msg::PointField__UINT32);
    descriptor10.count(1);
    offset += sizeof(uint32_t);

    /// @note Input data is 4 floats per point, input width is number of data elements (number of floats)
    const size_t cloud_width  = width / 4;

    _impl->_lidar.header(std::move(header));
    _impl->_lidar.width(cloud_width);
    _impl->_lidar.height(height);
    _impl->_lidar.is_bigendian(false);
    _impl->_lidar.fields({
      descriptor1,
      descriptor2,
      descriptor3,
      descriptor4,
      descriptor5,
      descriptor6,
      descriptor7,
      descriptor8,
      descriptor9,
      descriptor10
    });
    _impl->_lidar.point_step(offset);
    _impl->_lidar.row_step(cloud_width * offset);  // number of points * size of one point
    _impl->_lidar.is_dense(false); //True if there are not invalid points

    struct PointEx {
      float x{0.0f};
      float y{0.0f};
      float z{0.0f};
      uint8_t intensity{0U};
      uint8_t return_type{0U};
      uint16_t channel{0U};
      float azimuth{0.0f};
      float elevation{0.0f};
      float distance{0.0f};
      uint32_t time_stamp{0U};
    };

    // Validate whether sizes match
    if (sizeof(PointEx) != offset) {
      throw std::runtime_error([] {
        std::ostringstream oss;
        oss << __FILE__ << ":" << __LINE__ << " LiDAR extended data sizes don't match!";
        return oss.str();
      }());
    }

    std::vector<PointEx> data_ex;
    data_ex.reserve(height * width);
    for (auto it = data; it < data + height * width; it += 4) {
      auto & point = data_ex.emplace_back();
      point.x = *(it);
      point.y = *(it + 1);
      point.z = *(it + 2);
      point.intensity = static_cast<decltype(point.intensity)>(*(it + 3));
      // Leave the other members empty and use structure size to easy cast
    }

    // Assign channel
    size_t accumulated_size = 0;
    for (size_t header_idx = 0; header_idx < header_size; ++header_idx) {
      for (size_t point_idx = 0; point_idx < header_data[header_idx]; ++point_idx) {
        data_ex.at(accumulated_size + point_idx).channel = header_idx;
        data_ex.at(accumulated_size + point_idx).elevation = vertical_angles.at(header_idx);
      }
      accumulated_size += header_data[header_idx];
    }

    // Compute missing data from Cartesian data
    const uint32_t time_stamp = seconds * static_cast<uint32_t>(1e+9) + nanoseconds;
    for (auto & point : data_ex) {
      point.distance = std::hypot(point.x, point.y, point.z);
      point.azimuth = std::atan2(point.y, point.x);
      point.time_stamp = time_stamp;
    }

    std::vector<uint8_t> data_ex_raw;
    const std::size_t data_ex_raw_size = data_ex.size() * sizeof(PointEx);
    data_ex_raw.resize(data_ex_raw_size);
    std::memcpy(data_ex_raw.data(), data_ex.data(), data_ex_raw_size);

    _impl->_lidar.data(std::move(data_ex_raw));
  }

  CarlaLidarPublisher::CarlaLidarPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaLidarPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaLidarPublisher::~CarlaLidarPublisher() {
      if (!_impl)
          return;

      if (_impl->_datawriter)
          _impl->_publisher->delete_datawriter(_impl->_datawriter);

      if (_impl->_publisher)
          _impl->_participant->delete_publisher(_impl->_publisher);

      if (_impl->_topic)
          _impl->_participant->delete_topic(_impl->_topic);

      if (_impl->_participant)
          efd::DomainParticipantFactory::get_instance()->delete_participant(_impl->_participant);
  }

  CarlaLidarPublisher::CarlaLidarPublisher(const CarlaLidarPublisher& other) {
    _frame_id = other._frame_id;
    _name = other._name;
    _parent = other._parent;
    _impl = other._impl;
  }

  CarlaLidarPublisher& CarlaLidarPublisher::operator=(const CarlaLidarPublisher& other) {
    _frame_id = other._frame_id;
    _name = other._name;
    _parent = other._parent;
    _impl = other._impl;

    return *this;
  }

  CarlaLidarPublisher::CarlaLidarPublisher(CarlaLidarPublisher&& other) {
    _frame_id = std::move(other._frame_id);
    _name = std::move(other._name);
    _parent = std::move(other._parent);
    _impl = std::move(other._impl);
  }

  CarlaLidarPublisher& CarlaLidarPublisher::operator=(CarlaLidarPublisher&& other) {
    _frame_id = std::move(other._frame_id);
    _name = std::move(other._name);
    _parent = std::move(other._parent);
    _impl = std::move(other._impl);

    return *this;
  }
}
}
