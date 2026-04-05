#include "CarlaIMUPublisher.h"

#include <string>
#include <cmath>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "Imu.h"

namespace carla {
namespace ros2 {

  struct CarlaIMUPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    sensor_msgs_msg_Imu _imu {};
    std::string _frame_id_store;
  };

  bool CarlaIMUPublisher::Init() {
    _impl->_participant = dds_create_participant(GetDomainId(), nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;

    if (const auto custom_topic_name = ValidTopicName()) {
        topic_name = custom_topic_name.value();
    }
    topic_name = SanitizeTopicName(topic_name);
    _impl->_topic = dds_create_topic(_impl->_participant, &sensor_msgs_msg_Imu_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaIMUPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_imu) >= 0;
  }

  void CarlaIMUPublisher::SetData(int32_t seconds, uint32_t nanoseconds, float* pAccelerometer, float* pGyroscope, float compass) {
    geometry_msgs_msg_Vector3 gyroscope;
    geometry_msgs_msg_Vector3 linear_acceleration;
    const float ax = *pAccelerometer++;
    const float ay = *pAccelerometer++;
    const float az = *pAccelerometer++;
    linear_acceleration.x = ax;
    linear_acceleration.y = -ay;
    linear_acceleration.z = az;
    const float gx = *pGyroscope++;
    const float gy = *pGyroscope++;
    const float gz = *pGyroscope++;
    gyroscope.x = gx;
    gyroscope.y = -gy; // Invert pitch and yaw to match ROS
    gyroscope.z = -gz;

    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    std_msgs_msg_Header header;
    header.stamp = time;
    _impl->_frame_id_store = _frame_id;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    geometry_msgs_msg_Quaternion orientation;

    const float rx = 0.0f;                           // pitch
    const float ry = (float(M_PI_2) / 2.0f) - compass;  // yaw
    const float rz = 0.0f;                           // roll

    const float cr = cosf(rz * 0.5f);
    const float sr = sinf(rz * 0.5f);
    const float cp = cosf(rx * 0.5f);
    const float sp = sinf(rx * 0.5f);
    const float cy = cosf(ry * 0.5f);
    const float sy = sinf(ry * 0.5f);

    orientation.w = cr * cp * cy + sr * sp * sy;
    orientation.x = sr * cp * cy - cr * sp * sy;
    orientation.y = cr * sp * cy + sr * cp * sy;
    orientation.z = cr * cp * sy - sr * sp * cy;

    _impl->_imu.header = header;
    _impl->_imu.orientation = orientation;
    _impl->_imu.angular_velocity = gyroscope;
    _impl->_imu.linear_acceleration = linear_acceleration;
  }

  CarlaIMUPublisher::CarlaIMUPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaIMUPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaIMUPublisher::~CarlaIMUPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaIMUPublisher::CarlaIMUPublisher(const CarlaIMUPublisher&) = default;
  CarlaIMUPublisher& CarlaIMUPublisher::operator=(const CarlaIMUPublisher&) = default;
  CarlaIMUPublisher::CarlaIMUPublisher(CarlaIMUPublisher&&) = default;
  CarlaIMUPublisher& CarlaIMUPublisher::operator=(CarlaIMUPublisher&&) = default;
}
}
