#include "AutowarePublisher.h"
#include <string>

#include "carla/ros2/types/VelocityReport.h"
#include "carla/ros2/types/VelocityReportPubSubTypes.h"
#include "carla/ros2/types/SteeringReport.h"
#include "carla/ros2/types/SteeringReportPubSubTypes.h"
#include "carla/ros2/types/ControlModeReport.h"
#include "carla/ros2/types/ControlModeReportPubSubTypes.h"
#include "carla/ros2/types/GearReport.h"
#include "carla/ros2/types/GearReportPubSubTypes.h"
#include "carla/ros2/types/TurnIndicatorsReport.h"
#include "carla/ros2/types/TurnIndicatorsReportPubSubTypes.h"
#include "carla/ros2/types/HazardLightsReport.h"
#include "carla/ros2/types/HazardLightsReportPubSubTypes.h"

#include "AutowarePublisherBase.hpp"


namespace carla {
namespace ros2 {

// Generic traits template (forward-declared, specialized below)
template <typename Msg>
struct PublisherTraits;

// Specializations for each Autoware report type

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::VelocityReport> {
  using PubSub = autoware_vehicle_msgs::msg::VelocityReportPubSubType;
  static constexpr const char* topic = "/vehicle/status/velocity_status";
  static constexpr const char* human_name = "Autoware velocity report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::SteeringReport> {
  using PubSub = autoware_vehicle_msgs::msg::SteeringReportPubSubType;
  static constexpr const char* topic = "/vehicle/status/steering_status";
  static constexpr const char* human_name = "Autoware steering report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::ControlModeReport> {
  using PubSub = autoware_vehicle_msgs::msg::ControlModeReportPubSubType;
  static constexpr const char* topic = "/vehicle/status/control_mode";
  static constexpr const char* human_name = "Autoware control mode report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::GearReport> {
  using PubSub = autoware_vehicle_msgs::msg::GearReportPubSubType;
  static constexpr const char* topic = "/vehicle/status/gear_status";
  static constexpr const char* human_name = "Autoware gear report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::TurnIndicatorsReport> {
  using PubSub = autoware_vehicle_msgs::msg::TurnIndicatorsReportPubSubType;
  static constexpr const char* topic = "/vehicle/status/turn_indicators_status";
  static constexpr const char* human_name = "Autoware turn indicators report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::HazardLightsReport> {
  using PubSub = autoware_vehicle_msgs::msg::HazardLightsReportPubSubType;
  static constexpr const char* topic = "/vehicle/status/hazard_lights_status";
  static constexpr const char* human_name = "Autoware hazard lights report";
};

template <typename Msg>
class ReportPublisher
  : public AutowarePublisherBase<Msg, typename PublisherTraits<Msg>::PubSub> {
public:
  using Base = AutowarePublisherBase<Msg, typename PublisherTraits<Msg>::PubSub>;

  ReportPublisher()
    : Base("", "", PublisherTraits<Msg>::topic) {}

  const char* type() const override {
    return PublisherTraits<Msg>::human_name;
  }
};

// Type aliases for convenience
using VelocityReportPublisher =
  ReportPublisher<autoware_vehicle_msgs::msg::VelocityReport>;

using SteeringReportPublisher =
  ReportPublisher<autoware_vehicle_msgs::msg::SteeringReport>;

using ControlModeReportPublisher =
  ReportPublisher<autoware_vehicle_msgs::msg::ControlModeReport>;

using GearReportPublisher =
  ReportPublisher<autoware_vehicle_msgs::msg::GearReport>;

using TurnIndicatorsReportPublisher =
  ReportPublisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>;

using HazardLightsReportPublisher =
  ReportPublisher<autoware_vehicle_msgs::msg::HazardLightsReport>;


class AutowarePublisher::Implementation
{
public:
  VelocityReportPublisher       _velocity_publisher{};
  SteeringReportPublisher       _steering_publisher{};
  ControlModeReportPublisher    _control_mode_publisher{};
  GearReportPublisher           _gear_publisher{};
  TurnIndicatorsReportPublisher _turn_indicator_publisher{};
  HazardLightsReportPublisher   _hazard_lights_publisher{};
};

AutowarePublisher::AutowarePublisher([[maybe_unused]]void* vehicle, const CarlaPublisher::DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  _impl->_velocity_publisher.Init(domain_id);
  _impl->_steering_publisher.Init(domain_id);
  _impl->_control_mode_publisher.Init(domain_id);
  _impl->_gear_publisher.Init(domain_id);
  _impl->_turn_indicator_publisher.Init(domain_id);
  _impl->_hazard_lights_publisher.Init(domain_id);
  // TODO: Set QoS
}

void AutowarePublisher::SetVelocity(const float longitudinal_velocity, const float lateral_velocity, const float heading_rate)
{
  /// @note Leave `stamp` in header empty, because Publish() populates stamps
  std_msgs::msg::Header header;
  header.frame_id(_impl->_velocity_publisher.frame_id());

  autoware_vehicle_msgs::msg::VelocityReport report;
  report.header(std::move(header));
  report.longitudinal_velocity(longitudinal_velocity);
  report.lateral_velocity(lateral_velocity);
  report.heading_rate(heading_rate);

  _impl->_velocity_publisher.SetData(report);
}

void AutowarePublisher::SetSteering(const float steering_tire_angle)
{
  autoware_vehicle_msgs::msg::SteeringReport report;
  report.steering_tire_angle(steering_tire_angle);

  _impl->_steering_publisher.SetData(report);
}

void AutowarePublisher::SetControlMode(const uint8_t mode)
{
  autoware_vehicle_msgs::msg::ControlModeReport report;
  report.mode(mode);

  _impl->_control_mode_publisher.SetData(report);
}

void AutowarePublisher::SetGear(const uint8_t gear)
{
  autoware_vehicle_msgs::msg::GearReport report;
  report.report(gear);

  _impl->_gear_publisher.SetData(report);
}

void AutowarePublisher::SetTurnIndicators(const uint8_t turn_indicators)
{
  autoware_vehicle_msgs::msg::TurnIndicatorsReport report;
  report.report(turn_indicators);

  _impl->_turn_indicator_publisher.SetData(report);
}

void AutowarePublisher::SetHazardLights(const uint8_t hazard_lights)
{
  autoware_vehicle_msgs::msg::HazardLightsReport report;
  report.report(hazard_lights);

  _impl->_hazard_lights_publisher.SetData(report);
}

void AutowarePublisher::Publish(const int32_t seconds, const uint32_t nanoseconds)
{
  /**
   * @note All publishing is done at once based on assumption that all reports should be published
   * with the same frequency.
   * @sa https://github.com/tier4/AWSIM/blob/v1.3.1/docs/Components/ROS2/ROS2TopicAndServiceList/index.md
   */

  const auto current_time = [seconds, nanoseconds] {
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    return time;
  }();

  _impl->_velocity_publisher.Data().header().stamp(current_time);
  _impl->_steering_publisher.Data().stamp(current_time);
  _impl->_control_mode_publisher.Data().stamp(current_time);
  _impl->_gear_publisher.Data().stamp(current_time);
  _impl->_turn_indicator_publisher.Data().stamp(current_time);
  _impl->_hazard_lights_publisher.Data().stamp(current_time);

  _impl->_velocity_publisher.Publish();
  _impl->_steering_publisher.Publish();
  _impl->_control_mode_publisher.Publish();
  _impl->_gear_publisher.Publish();
  _impl->_turn_indicator_publisher.Publish();
  _impl->_hazard_lights_publisher.Publish();
}

}  // namespace carla
}  // namespace ros2
