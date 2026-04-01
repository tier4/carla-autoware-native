#include "FastDDSSubscriberListenerBase.h"
#include <iostream>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

namespace carla {
namespace ros2 {

namespace efd = eprosima::fastdds::dds;
using erc = eprosima::fastrtps::types::ReturnCode_t;

template<typename Message>
class SubscriberListenerBase<Message>::Implementation : public efd::DataReaderListener {
public:
  Implementation(Data* data) : _data(data) {}

  void on_subscription_matched(efd::DataReader* reader, const efd::SubscriptionMatchedStatus& info) override {
    if (info.current_count_change == 1) {
      _matched = info.total_count;
      _first_connected = true;
    } else if (info.current_count_change == -1) {
      _matched = info.total_count;
      if (_matched == 0) {
        // TODO: do sth here?
      }
    } else {
      std::cerr << info.current_count_change
                << " is not a valid value for PublicationMatchedStatus current count change"
                << std::endl;
    }
  }

  void on_data_available(efd::DataReader* reader) override {
    efd::SampleInfo info;
    erc rcode = reader->take_next_sample(&_message, &info);

    if (rcode == erc::ReturnCodeValue::RETCODE_OK) {
      if (info.instance_state == efd::ALIVE_INSTANCE_STATE) {
        std::unique_lock lock{_data->mutex};
        _data->data = _message;
        _data->data_changed = true;
      }
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ERROR) {
      std::cerr << "RETCODE_ERROR" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_UNSUPPORTED) {
      std::cerr << "RETCODE_UNSUPPORTED" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_BAD_PARAMETER) {
      std::cerr << "RETCODE_BAD_PARAMETER" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_PRECONDITION_NOT_MET) {
      std::cerr << "RETCODE_PRECONDITION_NOT_MET" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_OUT_OF_RESOURCES) {
      std::cerr << "RETCODE_OUT_OF_RESOURCES" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NOT_ENABLED) {
      std::cerr << "RETCODE_NOT_ENABLED" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_IMMUTABLE_POLICY) {
      std::cerr << "RETCODE_IMMUTABLE_POLICY" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_INCONSISTENT_POLICY) {
      std::cerr << "RETCODE_INCONSISTENT_POLICY" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ALREADY_DELETED) {
      std::cerr << "RETCODE_ALREADY_DELETED" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_TIMEOUT) {
      std::cerr << "RETCODE_TIMEOUT" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NO_DATA) {
      std::cerr << "RETCODE_NO_DATA" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ILLEGAL_OPERATION) {
      std::cerr << "RETCODE_ILLEGAL_OPERATION" << std::endl;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NOT_ALLOWED_BY_SECURITY) {
      std::cerr << "RETCODE_NOT_ALLOWED_BY_SECURITY" << std::endl;
    }
  }

  int _matched{0};
  bool _first_connected{false};
  Data* _data{nullptr};
  Message _message{};
};

template<typename Message>
SubscriberListenerBase<Message>::SubscriberListenerBase(Data* data)
  : _impl(std::make_unique<Implementation>(data)) {}

template<typename Message>
SubscriberListenerBase<Message>::~SubscriberListenerBase() = default;

template<typename Message>
void SubscriberListenerBase<Message>::SetOwnerData(Data* data) {
  _impl->_data = data;
}

}  // namespace ros2
}  // namespace carla


// Instantiate all types
#include "carla/ros2/types/Control.h"
#include "carla/ros2/types/GearCommand.h"
#include "carla/ros2/types/TurnIndicatorsCommand.h"
#include "carla/ros2/types/HazardLightsCommand.h"
#include "carla/ros2/types/VehicleEmergencyStamped.h"
#include "carla/ros2/types/Engage.h"

namespace carla {
namespace ros2 {

template class SubscriberListenerBase<autoware_control_msgs::msg::Control              >;
template class SubscriberListenerBase<autoware_vehicle_msgs::msg::GearCommand          >;
template class SubscriberListenerBase<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>;
template class SubscriberListenerBase<autoware_vehicle_msgs::msg::HazardLightsCommand  >;
template class SubscriberListenerBase<tier4_vehicle_msgs::msg::VehicleEmergencyStamped >;
template class SubscriberListenerBase<autoware_vehicle_msgs::msg::Engage               >;

}  // namespace ros2
}  // namespace carla
