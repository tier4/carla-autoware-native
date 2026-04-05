#include "CarlaTransformPublisher.h"

#include <string>
#include <vector>
#include <cstring>
#include <cmath>
#include <iostream>

#include "dds/dds.h"
#include "carla/ros2/cyclonedds/CycloneDDSTopicHelper.h"
#include "TFMessage.h"

namespace carla {
namespace ros2 {

  struct CarlaTransformPublisherImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _writer { 0 };
    tf2_msgs_msg_TFMessage _transform {};

    float last_translation[3] = {0.0f};
    float last_rotation[3] = {0.0f};
    geometry_msgs_msg_Vector3 vec_translation;
    geometry_msgs_msg_Quaternion vec_rotation;
    std::string _frame_id_store;
    std::string _child_frame_id_store;
    std::vector<geometry_msgs_msg_TransformStamped> _transforms_store;
  };

  bool CarlaTransformPublisher::Init() {
    _impl->_participant = dds_create_participant(GetDomainId(), nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string topic_name { "rt/tf" };
    _impl->_topic = dds_create_topic(_impl->_participant, &tf2_msgs_msg_TFMessage_desc, topic_name.c_str(), nullptr, nullptr);
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

  bool CarlaTransformPublisher::Publish() {
    return dds_write(_impl->_writer, &_impl->_transform) >= 0;
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

  CarlaTransformPublisher::~CarlaTransformPublisher() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaTransformPublisher::CarlaTransformPublisher(const CarlaTransformPublisher&) = default;
  CarlaTransformPublisher& CarlaTransformPublisher::operator=(const CarlaTransformPublisher&) = default;
  CarlaTransformPublisher::CarlaTransformPublisher(CarlaTransformPublisher&&) = default;
  CarlaTransformPublisher& CarlaTransformPublisher::operator=(CarlaTransformPublisher&&) = default;
}
}
