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

#include "carla/ros2/AutowareSteeringCompensation.h"

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
  static constexpr const char* frame_id = "base_link";
  static constexpr const char* topic = "/vehicle/status/velocity_status";
  static constexpr const char* human_name = "Autoware velocity report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::SteeringReport> {
  using PubSub = autoware_vehicle_msgs::msg::SteeringReportPubSubType;
  static constexpr const char* frame_id = "base_link";
  static constexpr const char* topic = "/vehicle/status/steering_status";
  static constexpr const char* human_name = "Autoware steering report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::ControlModeReport> {
  using PubSub = autoware_vehicle_msgs::msg::ControlModeReportPubSubType;
  static constexpr const char* frame_id = "base_link";
  static constexpr const char* topic = "/vehicle/status/control_mode";
  static constexpr const char* human_name = "Autoware control mode report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::GearReport> {
  using PubSub = autoware_vehicle_msgs::msg::GearReportPubSubType;
  static constexpr const char* frame_id = "base_link";
  static constexpr const char* topic = "/vehicle/status/gear_status";
  static constexpr const char* human_name = "Autoware gear report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::TurnIndicatorsReport> {
  using PubSub = autoware_vehicle_msgs::msg::TurnIndicatorsReportPubSubType;
  static constexpr const char* frame_id = "base_link";
  static constexpr const char* topic = "/vehicle/status/turn_indicators_status";
  static constexpr const char* human_name = "Autoware turn indicators report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs::msg::HazardLightsReport> {
  using PubSub = autoware_vehicle_msgs::msg::HazardLightsReportPubSubType;
  static constexpr const char* frame_id = "base_link";
  static constexpr const char* topic = "/vehicle/status/hazard_lights_status";
  static constexpr const char* human_name = "Autoware hazard lights report";
};

template <typename Msg>
class ReportPublisher
  : public AutowarePublisherBase<Msg, typename PublisherTraits<Msg>::PubSub> {
public:
  using Base = AutowarePublisherBase<Msg, typename PublisherTraits<Msg>::PubSub>;

  ReportPublisher()
    : Base(PublisherTraits<Msg>::frame_id, "", PublisherTraits<Msg>::topic) {}

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

AutowarePublisher::AutowarePublisher([[maybe_unused]]void* vehicle, const DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  const auto topic_config = [domain_id] {
    TopicConfig config;

    config.domain_id = domain_id;
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;

    return config;
  }();

  _impl->_velocity_publisher.Init(topic_config);
  _impl->_steering_publisher.Init(topic_config);
  _impl->_control_mode_publisher.Init(topic_config);
  _impl->_gear_publisher.Init(topic_config);
  _impl->_turn_indicator_publisher.Init(topic_config);
  _impl->_hazard_lights_publisher.Init(topic_config);
}

void AutowarePublisher::SetVelocity(float longitudinal_velocity, float lateral_velocity, float heading_rate)
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

void AutowarePublisher::SetSteering(float steering_tire_angle)
{
  autoware_vehicle_msgs::msg::SteeringReport report;

  report.steering_tire_angle(autoware_steering_compensation::GetDesiredSteeringCompensationRatio(steering_tire_angle));

  _impl->_steering_publisher.SetData(report);
}

void AutowarePublisher::SetControlMode(const ControlMode mode)
{
  autoware_vehicle_msgs::msg::ControlModeReport report;

  switch (mode) {
#define CASE(DATUM)                                                              \
  case ControlMode::DATUM:                                                       \
    report.mode(autoware_vehicle_msgs::msg::ControlModeReport_Constants::DATUM); \
    break;                                                                       \
    static_assert(true, "")

    CASE(NO_COMMAND);
    CASE(AUTONOMOUS);
    CASE(AUTONOMOUS_STEER_ONLY);
    CASE(AUTONOMOUS_VELOCITY_ONLY);
    CASE(MANUAL);
    CASE(DISENGAGED);
    CASE(NOT_READY);

#undef CASE
  }

  _impl->_control_mode_publisher.SetData(report);
}

void AutowarePublisher::SetGear(const Gear gear)
{
  autoware_vehicle_msgs::msg::GearReport report;

  switch (gear) {
#define CASE(DATUM)                                                         \
  case Gear::DATUM:                                                         \
    report.report(autoware_vehicle_msgs::msg::GearReport_Constants::DATUM); \
    break;                                                                  \
    static_assert(true, "")

    CASE(NONE);
    CASE(NEUTRAL);
    CASE(DRIVE);
    CASE(DRIVE_2);
    CASE(DRIVE_3);
    CASE(DRIVE_4);
    CASE(DRIVE_5);
    CASE(DRIVE_6);
    CASE(DRIVE_7);
    CASE(DRIVE_8);
    CASE(DRIVE_9);
    CASE(DRIVE_10);
    CASE(DRIVE_11);
    CASE(DRIVE_12);
    CASE(DRIVE_13);
    CASE(DRIVE_14);
    CASE(DRIVE_15);
    CASE(DRIVE_16);
    CASE(DRIVE_17);
    CASE(DRIVE_18);
    CASE(REVERSE);
    CASE(REVERSE_2);
    CASE(PARK);
    CASE(LOW);
    CASE(LOW_2);

#undef CASE
  }

  _impl->_gear_publisher.SetData(report);
}

void AutowarePublisher::SetTurnIndicators(const TurnIndicatorsStatus status)
{
  autoware_vehicle_msgs::msg::TurnIndicatorsReport report;

  switch (status) {
    case TurnIndicatorsStatus::OFF:
      report.report(autoware_vehicle_msgs::msg::TurnIndicatorsReport_Constants::DISABLE);
      break;
    case TurnIndicatorsStatus::LEFT:
      report.report(autoware_vehicle_msgs::msg::TurnIndicatorsReport_Constants::ENABLE_LEFT);
      break;
    case TurnIndicatorsStatus::RIGHT:
      report.report(autoware_vehicle_msgs::msg::TurnIndicatorsReport_Constants::ENABLE_RIGHT);
      break;
  }

  _impl->_turn_indicator_publisher.SetData(report);
}

void AutowarePublisher::SetHazardLights(const bool hazard_lights_enabled)
{
  autoware_vehicle_msgs::msg::HazardLightsReport report;

  report.report(
    hazard_lights_enabled ? autoware_vehicle_msgs::msg::HazardLightsReport_Constants::ENABLE
                          : autoware_vehicle_msgs::msg::HazardLightsReport_Constants::DISABLE);

  _impl->_hazard_lights_publisher.SetData(report);
}

void AutowarePublisher::Publish(int32_t seconds, uint32_t nanoseconds)
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
