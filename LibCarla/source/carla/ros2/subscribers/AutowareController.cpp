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

#include <algorithm>
#include <cmath>
#include <vector>

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

AutowareController::AutowareController(void* actor, const DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  const auto topic_config = [domain_id] {
    TopicConfig config;

    config.domain_id = domain_id;
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::TRANSIENT_LOCAL;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;

    return config;
  }();

  _impl->_control_subscriber.Init(topic_config);
  _impl->_gear_subscriber.Init(topic_config);
  _impl->_turn_indicator_subscriber.Init(topic_config);
  _impl->_hazard_lights_subscriber.Init(topic_config);
  _impl->_emergency_subscriber.Init(topic_config);
  _impl->_engage_subscriber.Init(topic_config);

  _impl->_vehicle = actor;
}

bool AutowareController::HasNewControl() const {
  // Debug
  if constexpr (false) {
#define PRINT(SUBSCRIBER)                                         \
  if (_impl->SUBSCRIBER.HasNewMessage())                          \
    std::cerr << #SUBSCRIBER << " Has new message!" << std::endl; \
    static_assert(true, "")

    PRINT(_control_subscriber);
    PRINT(_gear_subscriber);
    PRINT(_turn_indicator_subscriber);
    PRINT(_hazard_lights_subscriber);
    PRINT(_emergency_subscriber);
    PRINT(_engage_subscriber);

#undef PRINT
  }

  return _impl->_control_subscriber.HasNewMessage() ||
         _impl->_gear_subscriber.HasNewMessage() ||
         _impl->_turn_indicator_subscriber.HasNewMessage() ||
         _impl->_hazard_lights_subscriber.HasNewMessage() ||
         _impl->_emergency_subscriber.HasNewMessage() ||
         _impl->_engage_subscriber.HasNewMessage();
}

VehicleAccelerationControl AutowareController::GetControl() {
  VehicleAccelerationControl control_out{};

  const auto control_in = _impl->_control_subscriber.GetMessage();
  const auto gear_in = _impl->_gear_subscriber.GetMessage();
  const auto turn_indicator_in = _impl->_turn_indicator_subscriber.GetMessage();
  const auto hazard_lights_in = _impl->_hazard_lights_subscriber.GetMessage();
  const auto emergency_in = _impl->_emergency_subscriber.GetMessage();
  const auto engage_in = _impl->_engage_subscriber.GetMessage();

  // Debug
  if constexpr (false) {
    std::cerr << "=== NEW CONTROL ===" << std::endl;

    std::cerr << "Control"
      << " " << control_in.longitudinal().velocity()
      << " " << control_in.longitudinal().acceleration()
      << " " << control_in.longitudinal().jerk()
      << " " << (control_in.longitudinal().is_defined_acceleration() ? "True" : "False")
      << " " << (control_in.longitudinal().is_defined_jerk() ? "True" : "False")
      << " " << control_in.lateral().steering_tire_angle()
      << " " << control_in.lateral().steering_tire_rotation_rate()
      << " " << (control_in.lateral().is_defined_steering_tire_rotation_rate() ? "True" : "False")
      << std::endl;

    std::cerr << "Gear"
      << " " << static_cast<int>(gear_in.command())
      << std::endl;

    std::cerr << "Turn indicator"
      << " " << static_cast<int>(turn_indicator_in.command())
      << std::endl;

    std::cerr << "Hazard lights"
      << " " << static_cast<int>(hazard_lights_in.command())
      << std::endl;

    std::cerr << "Emergency"
      << " " << (emergency_in.emergency() ? "True" : "False")
      << std::endl;

    std::cerr << "Engage"
      << " " << (engage_in.engage() ? "True" : "False")
      << std::endl;
  }

  // Longitudinal: use ONLY acceleration from /control/command/control_cmd to drive the vehicle.
  // Velocity and jerk from the message are intentionally ignored.
  control_out.acceleration = control_in.longitudinal().acceleration();

  /// @note Set lateral negative, because Carla treats positive as right and Autoware expects positive to represent left (all when moving forward)
  const auto raw_steering = -control_in.lateral().steering_tire_angle();

  // Autoware publishes steering_tire_angle [rad] (front wheel angle).
  // We reuse the same convention as Ackermann: pass tire angle as steering.
  control_out.steer = raw_steering;
  control_out.steer_speed = 0.0f;
  if (control_in.lateral().is_defined_steering_tire_rotation_rate()) {
    control_out.steer_speed = -control_in.lateral().steering_tire_rotation_rate();
  }

  // TODO: Use input from all subscribers to perform actions in simulation

  return control_out;
}

void* AutowareController::GetVehicle() {
  return _impl->_vehicle;
}

}  // namespace ros2
}  // namespace carla
