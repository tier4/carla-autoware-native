#include "CarlaTransformPublisher.h"

#include <string>
#include <vector>
#include <cstring>
#include <cmath>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "TFMessage.h"

namespace carla {
namespace ros2 {

  struct CarlaTransformPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    tf2_msgs_msg_TFMessage _transform {};

    float last_translation[3] = {0.0f};
    float last_rotation[3] = {0.0f};
    geometry_msgs_msg_Vector3 vec_translation;
    geometry_msgs_msg_Quaternion vec_rotation;
    std::string _frame_id_store;
    std::string _child_frame_id_store;
    std::vector<geometry_msgs_msg_TransformStamped> _transforms_store;
  };

  bool CarlaTransformPublisher::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSPublisher("tf2_msgs::msg::TFMessage");
    if (!_impl->_dds) return false;

    const std::string topic_name { "rt/tf" };

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaTransformPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_transform);
  }

  void CarlaTransformPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const float* translation, const float* rotation) {

    int same_translation = std::memcmp(translation, _impl->last_translation, sizeof(float) * 3);
    int same_rotation = std::memcmp(rotation, _impl->last_rotation, sizeof(float) * 3);
    if (same_translation != 0 || same_rotation != 0) {
        std::memcpy(_impl->last_translation, translation, sizeof(float) * 3);
        std::memcpy(_impl->last_rotation, rotation, sizeof(float) * 3);

        const float tx = *translation++;
        const float ty = *translation++;
        const float tz = *translation++;

        const float rx = ((*rotation++) * -1.0f) * (M_PIf32 / 180.0f);
        const float ry = ((*rotation++) * -1.0f) * (M_PIf32 / 180.0f);
        const float rz = *rotation++ * (M_PIf32 / 180.0f);

        const float cr = cosf(rz * 0.5f);
        const float sr = sinf(rz * 0.5f);
        const float cp = cosf(rx * 0.5f);
        const float sp = sinf(rx * 0.5f);
        const float cy = cosf(ry * 0.5f);
        const float sy = sinf(ry * 0.5f);

        _impl->vec_translation.x = tx;
        _impl->vec_translation.y = -ty;
        _impl->vec_translation.z = tz;

        _impl->vec_rotation.w = cr * cp * cy + sr * sp * sy;
        _impl->vec_rotation.x = sr * cp * cy - cr * sp * sy;
        _impl->vec_rotation.y = cr * sp * cy + sr * cp * sy;
        _impl->vec_rotation.z = cr * cp * sy - sr * sp * cy;
    }

    builtin_interfaces_msg_Time time;
    time.sec = seconds;
    time.nanosec = nanoseconds;

    _impl->_frame_id_store = _parent;
    std_msgs_msg_Header header;
    header.stamp = time;
    header.frame_id = const_cast<char*>(_impl->_frame_id_store.c_str());

    geometry_msgs_msg_Transform t;
    t.rotation = _impl->vec_rotation;
    t.translation = _impl->vec_translation;

    _impl->_child_frame_id_store = _frame_id;
    geometry_msgs_msg_TransformStamped ts;
    ts.header = header;
    ts.transform = t;
    ts.child_frame_id = const_cast<char*>(_impl->_child_frame_id_store.c_str());
    _impl->_transforms_store = {ts};
    _impl->_transform.transforms._buffer = _impl->_transforms_store.data();
    _impl->_transform.transforms._length = 1;
    _impl->_transform.transforms._maximum = 1;
    _impl->_transform.transforms._release = false;
  }

  CarlaTransformPublisher::CarlaTransformPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaTransformPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaTransformPublisher::~CarlaTransformPublisher() = default;
  CarlaTransformPublisher::CarlaTransformPublisher(const CarlaTransformPublisher&) = default;
  CarlaTransformPublisher& CarlaTransformPublisher::operator=(const CarlaTransformPublisher&) = default;
  CarlaTransformPublisher::CarlaTransformPublisher(CarlaTransformPublisher&&) = default;
  CarlaTransformPublisher& CarlaTransformPublisher::operator=(CarlaTransformPublisher&&) = default;
}
}
