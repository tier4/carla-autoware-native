#include "AutowareController.h"

#include "carla/ros2/types/Control.h"
#include "carla/ros2/types/ControlPubSubTypes.h"
#include "carla/ros2/types/GearCommand.h"
#include "carla/ros2/types/GearCommandPubSubTypes.h"
#include "carla/ros2/types/TurnIndicatorsCommand.h"
#include "carla/ros2/types/TurnIndicatorsCommandPubSubTypes.h"
#include "carla/ros2/types/HazardLightsCommand.h"
#include "carla/ros2/types/HazardLightsCommandPubSubTypes.h"
#include "carla/ros2/types/VehicleEmergencyStamped.h"
#include "carla/ros2/types/VehicleEmergencyStampedPubSubTypes.h"
#include "carla/ros2/types/Engage.h"
#include "carla/ros2/types/EngagePubSubTypes.h"

#include "carla/ros2/subscribers/AutowareSubscriber.h"

namespace carla {
namespace ros2 {

// Define subscribers
class AutowareControlSubscriber
: public AutowareSubscriber<autoware_control_msgs::msg::Control, autoware_control_msgs::msg::ControlPubSubType>
{
public:
  AutowareControlSubscriber() : AutowareSubscriber("", "", "/control/command/control_cmd") {}

  virtual const char* type() const override { return "Autoware vehicle control"; }
};

class AutowareGearCommandSubscriber
: public AutowareSubscriber<autoware_vehicle_msgs::msg::GearCommand, autoware_vehicle_msgs::msg::GearCommandPubSubType>
{
public:
  AutowareGearCommandSubscriber() : AutowareSubscriber("", "", "/control/command/gear_cmd") {}

  virtual const char* type() const override { return "Autoware gear command"; }
};

class AutowareTurnIndicatorsCommandSubscriber
: public AutowareSubscriber<autoware_vehicle_msgs::msg::TurnIndicatorsCommand, autoware_vehicle_msgs::msg::TurnIndicatorsCommandPubSubType>
{
public:
  AutowareTurnIndicatorsCommandSubscriber() : AutowareSubscriber("", "", "/control/command/turn_indicators_cmd") {}

  virtual const char* type() const override { return "Autoware turn indicator command"; }
};

class AutowareHazardLightsCommandSubscriber
: public AutowareSubscriber<autoware_vehicle_msgs::msg::HazardLightsCommand, autoware_vehicle_msgs::msg::HazardLightsCommandPubSubType>
{
public:
  AutowareHazardLightsCommandSubscriber() : AutowareSubscriber("", "", "/control/command/hazard_lights_cmd") {}

  virtual const char* type() const override { return "Autoware hazard lights command"; }
};

class AutowareVehicleEmergencyStampedSubscriber
: public AutowareSubscriber<tier4_vehicle_msgs::msg::VehicleEmergencyStamped, tier4_vehicle_msgs::msg::VehicleEmergencyStampedPubSubType>
{
public:
  AutowareVehicleEmergencyStampedSubscriber() : AutowareSubscriber("", "", "/control/command/emergency_cmd") {}

  virtual const char* type() const override { return "Autoware emergency"; }
};

class AutowareEngageSubscriber
: public AutowareSubscriber<autoware_vehicle_msgs::msg::Engage, autoware_vehicle_msgs::msg::EngagePubSubType>
{
public:
  AutowareEngageSubscriber() : AutowareSubscriber("", "", "/vehicle/engage") {}

  virtual const char* type() const override { return "Autoware engage"; }
};


class AutowareController::Implementation {
public:
  Implementation() = default;

  AutowareControlSubscriber                 _control_subscriber{};
  AutowareGearCommandSubscriber             _gear_subscriber{};
  AutowareTurnIndicatorsCommandSubscriber   _turn_indicator_subscriber{};
  AutowareHazardLightsCommandSubscriber     _hazard_lights_subscriber{};
  AutowareVehicleEmergencyStampedSubscriber _emergency_subscriber{};
  AutowareEngageSubscriber                  _engage_subscriber{};

  void* _vehicle;
};

AutowareController::AutowareController(void* actor, const CarlaSubscriber::DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  const auto subscriber_config = [domain_id] {
    AutowareSubscriberConfig config;

    config.domain_id = domain_id;
    config.reliability_qos = AutowareSubscriberConfig::ReliabilityQoS::RELIABLE;
    config.durability_qos = AutowareSubscriberConfig::DurabilityQoS::TRANSIENT_LOCAL;
    config.history_qos = AutowareSubscriberConfig::HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;

    return config;
  }();

  _impl->_control_subscriber.Init(subscriber_config);
  _impl->_gear_subscriber.Init(subscriber_config);
  _impl->_turn_indicator_subscriber.Init(subscriber_config);
  _impl->_hazard_lights_subscriber.Init(subscriber_config);
  _impl->_emergency_subscriber.Init(subscriber_config);
  _impl->_engage_subscriber.Init(subscriber_config);

  _impl->_vehicle = actor;
}

bool AutowareController::HasNewControl() const {
  return _impl->_control_subscriber.HasNewMessage() ||
         _impl->_gear_subscriber.HasNewMessage() ||
         _impl->_turn_indicator_subscriber.HasNewMessage() ||
         _impl->_hazard_lights_subscriber.HasNewMessage() ||
         _impl->_emergency_subscriber.HasNewMessage() ||
         _impl->_engage_subscriber.HasNewMessage();
}

VehicleAckermannControl AutowareController::GetControl() {
  VehicleAckermannControl control_out;
  const auto control_in = _impl->_control_subscriber.GetMessage();

  control_out.speed = control_in.longitudinal().velocity();
  if (control_in.longitudinal().is_defined_acceleration()) {
    control_out.acceleration = control_in.longitudinal().acceleration();
  }
  if (control_in.longitudinal().is_defined_jerk()) {
    control_out.jerk = control_in.longitudinal().jerk();
  }

  /// @note Set lateral negative, because Carla treats positive as right and Autoware expects positive to represent left (all when moving forward)
  control_out.steer = -control_in.lateral().steering_tire_angle();
  if (control_in.lateral().is_defined_steering_tire_rotation_rate()) {
    control_out.steer_speed = -control_in.lateral().steering_tire_rotation_rate();
  }

  return control_out;
}

void* AutowareController::GetVehicle() {
  return _impl->_vehicle;
}

}  // namespace ros2
}  // namespace carla
