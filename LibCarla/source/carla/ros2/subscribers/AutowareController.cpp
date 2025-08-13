#include "AutowareController.h"

#include "carla/ros2/types/Control.h"
#include "carla/ros2/types/ControlPubSubTypes.h"
#include "carla/ros2/subscribers/AutowareSubscriber.h"

namespace carla {
namespace ros2 {

// Define subscribers
class AutowareControlSubscriber
: public AutowareSubscriber<autoware_control_msgs::msg::Control, autoware_control_msgs::msg::ControlPubSubType>
{
public:
  AutowareControlSubscriber() : AutowareSubscriber("ros_name", "parent", "ros_topic_name") {}

  virtual const char* type() const override { return "Autoware vehicle control"; }
};


class AutowareController::Implementation {
public:
  Implementation() = default;
  AutowareControlSubscriber _control_subscriber{};
  void* _vehicle;
};

AutowareController::AutowareController(void* actor, const CarlaSubscriber::DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  AutowareSubscriberConfig config;
  config.domain_id = domain_id;
  _impl->_control_subscriber.Init(config);
  _impl->_vehicle = actor;
}

bool AutowareController::HasNewControl() const {
  return _impl->_control_subscriber.HasNewMessage();
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
