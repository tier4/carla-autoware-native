#include "CarlaIMUPublisher.h"

#include <string>
#include <cmath>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Imu.h"

namespace carla {
namespace ros2 {

  struct CarlaIMUPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    sensor_msgs_msg_Imu _imu {};
    std::string _frame_id_store;
  };

  bool CarlaIMUPublisher::Init(const TopicConfig& config) {
    _impl->_dds = CreateDDSPublisher("sensor_msgs::msg::Imu");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += config.suffix;
    if (const auto custom_topic_name = ValidTopicName(config.suffix)) {
      topic_name = custom_topic_name.value();
    }

    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaIMUPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_imu);
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

    _impl->_frame_id_store = _frame_id;
    std_msgs_msg_Header header;
    header.stamp = time;
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

  CarlaIMUPublisher::CarlaIMUPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaIMUPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaIMUPublisher::~CarlaIMUPublisher() = default;
  CarlaIMUPublisher::CarlaIMUPublisher(const CarlaIMUPublisher&) = default;
  CarlaIMUPublisher& CarlaIMUPublisher::operator=(const CarlaIMUPublisher&) = default;
  CarlaIMUPublisher::CarlaIMUPublisher(CarlaIMUPublisher&&) = default;
  CarlaIMUPublisher& CarlaIMUPublisher::operator=(CarlaIMUPublisher&&) = default;
}
}
