#include "CarlaEgoVehicleControlSubscriber.h"

#include <string>
#include <iostream>

#include "dds/dds.h"
#include "CarlaEgoVehicleControl.h"

namespace carla {
namespace ros2 {

  struct CarlaEgoVehicleControlSubscriberImpl {
    dds_entity_t _participant { 0 };
    dds_entity_t _topic { 0 };
    dds_entity_t _reader { 0 };
    carla_msgs_msg_CarlaEgoVehicleControl _event {};
    VehicleControl _control {};
    bool _new_message {false};
    bool _alive {true};
    void* _vehicle {nullptr};
  };

  static void on_vehicle_control_data(dds_entity_t reader, void* arg) {
    CarlaEgoVehicleControlSubscriber* self = static_cast<CarlaEgoVehicleControlSubscriber*>(arg);
    if (!self) return;

    carla_msgs_msg_CarlaEgoVehicleControl msg {};
    void* samples[1] = { &msg };
    dds_sample_info_t infos[1];

    int32_t n = dds_take(reader, samples, infos, 1, 1);
    if (n > 0 && infos[0].valid_data) {
      VehicleControl control;
      control.throttle = msg.throttle;
      control.steer = msg.steer;
      control.brake = msg.brake;
      control.hand_brake = msg.hand_brake;
      control.reverse = msg.reverse;
      control.gear = msg.gear;
      control.manual_gear_shift = msg.manual_gear_shift;
      self->ForwardMessage(control);
    }
  }

  bool CarlaEgoVehicleControlSubscriber::Init() {
    _impl->_participant = dds_create_participant(0, nullptr, nullptr);
    if (_impl->_participant < 0) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    const std::string base { "rt/carla/" };
    const std::string publisher_type {"/vehicle_control_cmd"};
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    topic_name += publisher_type;

    _impl->_topic = dds_create_topic(_impl->_participant, &carla_msgs_msg_CarlaEgoVehicleControl_desc, topic_name.c_str(), nullptr, nullptr);
    if (_impl->_topic < 0) {
        std::cerr << "Failed to create Topic" << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    // Create listener for data available callback
    dds_listener_t* listener = dds_create_listener(this);
    dds_lset_data_available(listener, on_vehicle_control_data);

    _impl->_reader = dds_create_reader(_impl->_participant, _impl->_topic, nullptr, listener);
    dds_delete_listener(listener);
    if (_impl->_reader < 0) {
        std::cerr << "Failed to create DataReader" << std::endl;
        dds_delete(_impl->_participant);
        return false;
    }

    return true;
  }

  bool CarlaEgoVehicleControlSubscriber::Read() {
    void* samples[1] = { &_impl->_event };
    dds_sample_info_t infos[1];
    int32_t n = dds_take(_impl->_reader, samples, infos, 1, 1);
    return (n > 0 && infos[0].valid_data);
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

  CarlaEgoVehicleControlSubscriber::CarlaEgoVehicleControlSubscriber(void* vehicle, const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaEgoVehicleControlSubscriberImpl>()) {
    _impl->_vehicle = vehicle;
    _name = ros_name;
    _parent = parent;
  }

  CarlaEgoVehicleControlSubscriber::~CarlaEgoVehicleControlSubscriber() {
    if (_impl && _impl->_participant > 0)
        dds_delete(_impl->_participant);
  }

  CarlaEgoVehicleControlSubscriber::CarlaEgoVehicleControlSubscriber(const CarlaEgoVehicleControlSubscriber&) = default;
  CarlaEgoVehicleControlSubscriber& CarlaEgoVehicleControlSubscriber::operator=(const CarlaEgoVehicleControlSubscriber&) = default;
  CarlaEgoVehicleControlSubscriber::CarlaEgoVehicleControlSubscriber(CarlaEgoVehicleControlSubscriber&&) = default;
  CarlaEgoVehicleControlSubscriber& CarlaEgoVehicleControlSubscriber::operator=(CarlaEgoVehicleControlSubscriber&&) = default;
}
}
