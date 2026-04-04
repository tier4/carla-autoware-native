#include "CarlaEgoVehicleControlSubscriber.h"

#include "carla/ros2/dds/DDSSubscriberImpl.h"
#include "CarlaEgoVehicleControl.h"

namespace carla {
namespace ros2 {

  struct CarlaEgoVehicleControlSubscriberImpl {
    std::unique_ptr<DDSSubscriberImpl> _dds;
    carla_msgs_msg_CarlaEgoVehicleControl _event {};
    VehicleControl _control {};
    bool _new_message {false};
    bool _alive {true};
    void* _vehicle {nullptr};
  };

  bool CarlaEgoVehicleControlSubscriber::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSSubscriber("carla_msgs::msg::CarlaEgoVehicleControl");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    const std::string publisher_type {"/vehicle_control_cmd"};
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;
    if (const auto custom_topic_name = ValidTopicName(publisher_type)) {
      topic_name = custom_topic_name.value();
    }

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name)) {
      return false;
    }

    // Set up callback to replicate the old CarlaSubscriberListener behavior:
    // when data arrives, read it, convert to VehicleControl, and forward.
    _impl->_dds->SetOnDataCallback([this]() {
      if (_impl->_dds->TakeNextSample(&_impl->_event)) {
        VehicleControl control;
        control.throttle = _impl->_event.throttle;
        control.steer = _impl->_event.steer;
        control.brake = _impl->_event.brake;
        control.hand_brake = _impl->_event.hand_brake;
        control.reverse = _impl->_event.reverse;
        control.gear = _impl->_event.gear;
        control.manual_gear_shift = _impl->_event.manual_gear_shift;
        ForwardMessage(control);
      }
    });

    return true;
  }

  bool CarlaEgoVehicleControlSubscriber::Read() {
    return _impl->_dds->TakeNextSample(&_impl->_event);
  }

  void CarlaEgoVehicleControlSubscriber::ForwardMessage(VehicleControl control) {
    _impl->_control = control;
    _impl->_new_message = true;
  }

  void CarlaEgoVehicleControlSubscriber::DestroySubscriber() {
    _impl->_alive = false;
  }

  VehicleControl CarlaEgoVehicleControlSubscriber::GetMessage() {
    _impl->_new_message = false;
    return _impl->_control;
  }

  bool CarlaEgoVehicleControlSubscriber::IsAlive() {
    return _impl->_alive;
  }

  bool CarlaEgoVehicleControlSubscriber::HasNewMessage() {
    return _impl->_new_message;
  }

  void* CarlaEgoVehicleControlSubscriber::GetVehicle() {
    return _impl->_vehicle;
  }

  CarlaEgoVehicleControlSubscriber::CarlaEgoVehicleControlSubscriber(void* vehicle, const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaEgoVehicleControlSubscriberImpl>()) {
    _impl->_vehicle = vehicle;
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaEgoVehicleControlSubscriber::~CarlaEgoVehicleControlSubscriber() = default;
  CarlaEgoVehicleControlSubscriber::CarlaEgoVehicleControlSubscriber(const CarlaEgoVehicleControlSubscriber&) = default;
  CarlaEgoVehicleControlSubscriber& CarlaEgoVehicleControlSubscriber::operator=(const CarlaEgoVehicleControlSubscriber&) = default;
  CarlaEgoVehicleControlSubscriber::CarlaEgoVehicleControlSubscriber(CarlaEgoVehicleControlSubscriber&&) = default;
  CarlaEgoVehicleControlSubscriber& CarlaEgoVehicleControlSubscriber::operator=(CarlaEgoVehicleControlSubscriber&&) = default;
}
}
