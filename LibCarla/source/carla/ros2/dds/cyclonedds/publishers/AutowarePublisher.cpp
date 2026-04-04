#include "AutowarePublisher.h"
#include <string>

#include "VelocityReport.h"
#include "SteeringReport.h"
#include "ControlModeReport.h"
#include "GearReport.h"
#include "TurnIndicatorsReport.h"
#include "HazardLightsReport.h"

#include "carla/ros2/AutowareSteeringCompensation.h"

#include "AutowarePublisherBase.hpp"


namespace carla {
namespace ros2 {

// ---------------------------------------------------------------------------
// Compile-time type-name constants (inline constexpr char[] for non-type
// template parameters -- requires C++17).
// ---------------------------------------------------------------------------
inline constexpr char velocity_report_type[]        = "autoware_vehicle_msgs::msg::VelocityReport";
inline constexpr char steering_report_type[]        = "autoware_vehicle_msgs::msg::SteeringReport";
inline constexpr char control_mode_report_type[]    = "autoware_vehicle_msgs::msg::ControlModeReport";
inline constexpr char gear_report_type[]            = "autoware_vehicle_msgs::msg::GearReport";
inline constexpr char turn_indicators_report_type[] = "autoware_vehicle_msgs::msg::TurnIndicatorsReport";
inline constexpr char hazard_lights_report_type[]   = "autoware_vehicle_msgs::msg::HazardLightsReport";

// ---------------------------------------------------------------------------
// Autoware constant values (from the IDL / FastDDS Constants classes).
// CycloneDDS C headers do not expose named constants, so we define them here.
// ---------------------------------------------------------------------------

// ControlModeReport mode values
namespace ControlModeReportConstants {
  static constexpr uint8_t NO_COMMAND               = 0;
  static constexpr uint8_t AUTONOMOUS                = 1;
  static constexpr uint8_t AUTONOMOUS_STEER_ONLY     = 2;
  static constexpr uint8_t AUTONOMOUS_VELOCITY_ONLY  = 3;
  static constexpr uint8_t MANUAL                    = 4;
  static constexpr uint8_t DISENGAGED                = 5;
  static constexpr uint8_t NOT_READY                 = 6;
}

// GearReport report values
namespace GearReportConstants {
  static constexpr uint8_t NONE      = 0;
  static constexpr uint8_t NEUTRAL   = 1;
  static constexpr uint8_t DRIVE     = 2;
  static constexpr uint8_t DRIVE_2   = 3;
  static constexpr uint8_t DRIVE_3   = 4;
  static constexpr uint8_t DRIVE_4   = 5;
  static constexpr uint8_t DRIVE_5   = 6;
  static constexpr uint8_t DRIVE_6   = 7;
  static constexpr uint8_t DRIVE_7   = 8;
  static constexpr uint8_t DRIVE_8   = 9;
  static constexpr uint8_t DRIVE_9   = 10;
  static constexpr uint8_t DRIVE_10  = 11;
  static constexpr uint8_t DRIVE_11  = 12;
  static constexpr uint8_t DRIVE_12  = 13;
  static constexpr uint8_t DRIVE_13  = 14;
  static constexpr uint8_t DRIVE_14  = 15;
  static constexpr uint8_t DRIVE_15  = 16;
  static constexpr uint8_t DRIVE_16  = 17;
  static constexpr uint8_t DRIVE_17  = 18;
  static constexpr uint8_t DRIVE_18  = 19;
  static constexpr uint8_t REVERSE   = 20;
  static constexpr uint8_t REVERSE_2 = 21;
  static constexpr uint8_t PARK      = 22;
  static constexpr uint8_t LOW       = 23;
  static constexpr uint8_t LOW_2     = 24;
}

// TurnIndicatorsReport report values
namespace TurnIndicatorsReportConstants {
  static constexpr uint8_t DISABLE      = 1;
  static constexpr uint8_t ENABLE_LEFT  = 2;
  static constexpr uint8_t ENABLE_RIGHT = 3;
}

// HazardLightsReport report values
namespace HazardLightsReportConstants {
  static constexpr uint8_t DISABLE = 1;
  static constexpr uint8_t ENABLE  = 2;
}

// ---------------------------------------------------------------------------
// PublisherTraits: per-message-type metadata used by ReportPublisher.
// ---------------------------------------------------------------------------
template <typename Msg>
struct PublisherTraits;

template <>
struct PublisherTraits<autoware_vehicle_msgs_msg_VelocityReport> {
  static constexpr const char* type_name  = velocity_report_type;
  static constexpr const char* frame_id   = "base_link";
  static constexpr const char* topic      = "/vehicle/status/velocity_status";
  static constexpr const char* human_name = "Autoware velocity report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs_msg_SteeringReport> {
  static constexpr const char* type_name  = steering_report_type;
  static constexpr const char* frame_id   = "base_link";
  static constexpr const char* topic      = "/vehicle/status/steering_status";
  static constexpr const char* human_name = "Autoware steering report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs_msg_ControlModeReport> {
  static constexpr const char* type_name  = control_mode_report_type;
  static constexpr const char* frame_id   = "base_link";
  static constexpr const char* topic      = "/vehicle/status/control_mode";
  static constexpr const char* human_name = "Autoware control mode report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs_msg_GearReport> {
  static constexpr const char* type_name  = gear_report_type;
  static constexpr const char* frame_id   = "base_link";
  static constexpr const char* topic      = "/vehicle/status/gear_status";
  static constexpr const char* human_name = "Autoware gear report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs_msg_TurnIndicatorsReport> {
  static constexpr const char* type_name  = turn_indicators_report_type;
  static constexpr const char* frame_id   = "base_link";
  static constexpr const char* topic      = "/vehicle/status/turn_indicators_status";
  static constexpr const char* human_name = "Autoware turn indicators report";
};

template <>
struct PublisherTraits<autoware_vehicle_msgs_msg_HazardLightsReport> {
  static constexpr const char* type_name  = hazard_lights_report_type;
  static constexpr const char* frame_id   = "base_link";
  static constexpr const char* topic      = "/vehicle/status/hazard_lights_status";
  static constexpr const char* human_name = "Autoware hazard lights report";
};

// ---------------------------------------------------------------------------
// ReportPublisher: concrete publisher instantiated per report type.
// ---------------------------------------------------------------------------
template <typename Msg>
class ReportPublisher
  : public AutowarePublisherBase<Msg, PublisherTraits<Msg>::type_name> {
public:
  using Base = AutowarePublisherBase<Msg, PublisherTraits<Msg>::type_name>;

  ReportPublisher()
    : Base(PublisherTraits<Msg>::frame_id, "", PublisherTraits<Msg>::topic) {}

  const char* type() const override {
    return PublisherTraits<Msg>::human_name;
  }
};

// Type aliases
using VelocityReportPublisher =
  ReportPublisher<autoware_vehicle_msgs_msg_VelocityReport>;

using SteeringReportPublisher =
  ReportPublisher<autoware_vehicle_msgs_msg_SteeringReport>;

using ControlModeReportPublisher =
  ReportPublisher<autoware_vehicle_msgs_msg_ControlModeReport>;

using GearReportPublisher =
  ReportPublisher<autoware_vehicle_msgs_msg_GearReport>;

using TurnIndicatorsReportPublisher =
  ReportPublisher<autoware_vehicle_msgs_msg_TurnIndicatorsReport>;

using HazardLightsReportPublisher =
  ReportPublisher<autoware_vehicle_msgs_msg_HazardLightsReport>;


class AutowarePublisher::Implementation
{
public:
  VelocityReportPublisher       _velocity_publisher{};
  SteeringReportPublisher       _steering_publisher{};
  ControlModeReportPublisher    _control_mode_publisher{};
  GearReportPublisher           _gear_publisher{};
  TurnIndicatorsReportPublisher _turn_indicator_publisher{};
  HazardLightsReportPublisher   _hazard_lights_publisher{};

  // Backing store for the VelocityReport header.frame_id (char* field)
  std::string _velocity_frame_id_store;
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
  _impl->_velocity_frame_id_store = _impl->_velocity_publisher.frame_id();

  std_msgs_msg_Header header {};
  header.frame_id = const_cast<char*>(_impl->_velocity_frame_id_store.c_str());

  autoware_vehicle_msgs_msg_VelocityReport report {};
  report.header = header;
  report.longitudinal_velocity = longitudinal_velocity;
  report.lateral_velocity = lateral_velocity;
  report.heading_rate = heading_rate;

  _impl->_velocity_publisher.SetData(report);
}

void AutowarePublisher::SetSteering(float steering_tire_angle)
{
  autoware_vehicle_msgs_msg_SteeringReport report {};

  report.steering_tire_angle = autoware_steering_compensation::GetSteeringOutput(steering_tire_angle);

  _impl->_steering_publisher.SetData(report);
}

void AutowarePublisher::SetControlMode(const ControlMode mode)
{
  autoware_vehicle_msgs_msg_ControlModeReport report {};

  switch (mode) {
    case ControlMode::NO_COMMAND:
      report.mode = ControlModeReportConstants::NO_COMMAND;
      break;
    case ControlMode::AUTONOMOUS:
      report.mode = ControlModeReportConstants::AUTONOMOUS;
      break;
    case ControlMode::AUTONOMOUS_STEER_ONLY:
      report.mode = ControlModeReportConstants::AUTONOMOUS_STEER_ONLY;
      break;
    case ControlMode::AUTONOMOUS_VELOCITY_ONLY:
      report.mode = ControlModeReportConstants::AUTONOMOUS_VELOCITY_ONLY;
      break;
    case ControlMode::MANUAL:
      report.mode = ControlModeReportConstants::MANUAL;
      break;
    case ControlMode::DISENGAGED:
      report.mode = ControlModeReportConstants::DISENGAGED;
      break;
    case ControlMode::NOT_READY:
      report.mode = ControlModeReportConstants::NOT_READY;
      break;
  }

  _impl->_control_mode_publisher.SetData(report);
}

void AutowarePublisher::SetGear(const Gear gear)
{
  autoware_vehicle_msgs_msg_GearReport report {};

  switch (gear) {
    case Gear::NONE:      report.report = GearReportConstants::NONE;      break;
    case Gear::NEUTRAL:   report.report = GearReportConstants::NEUTRAL;   break;
    case Gear::DRIVE:     report.report = GearReportConstants::DRIVE;     break;
    case Gear::DRIVE_2:   report.report = GearReportConstants::DRIVE_2;   break;
    case Gear::DRIVE_3:   report.report = GearReportConstants::DRIVE_3;   break;
    case Gear::DRIVE_4:   report.report = GearReportConstants::DRIVE_4;   break;
    case Gear::DRIVE_5:   report.report = GearReportConstants::DRIVE_5;   break;
    case Gear::DRIVE_6:   report.report = GearReportConstants::DRIVE_6;   break;
    case Gear::DRIVE_7:   report.report = GearReportConstants::DRIVE_7;   break;
    case Gear::DRIVE_8:   report.report = GearReportConstants::DRIVE_8;   break;
    case Gear::DRIVE_9:   report.report = GearReportConstants::DRIVE_9;   break;
    case Gear::DRIVE_10:  report.report = GearReportConstants::DRIVE_10;  break;
    case Gear::DRIVE_11:  report.report = GearReportConstants::DRIVE_11;  break;
    case Gear::DRIVE_12:  report.report = GearReportConstants::DRIVE_12;  break;
    case Gear::DRIVE_13:  report.report = GearReportConstants::DRIVE_13;  break;
    case Gear::DRIVE_14:  report.report = GearReportConstants::DRIVE_14;  break;
    case Gear::DRIVE_15:  report.report = GearReportConstants::DRIVE_15;  break;
    case Gear::DRIVE_16:  report.report = GearReportConstants::DRIVE_16;  break;
    case Gear::DRIVE_17:  report.report = GearReportConstants::DRIVE_17;  break;
    case Gear::DRIVE_18:  report.report = GearReportConstants::DRIVE_18;  break;
    case Gear::REVERSE:   report.report = GearReportConstants::REVERSE;   break;
    case Gear::REVERSE_2: report.report = GearReportConstants::REVERSE_2; break;
    case Gear::PARK:      report.report = GearReportConstants::PARK;      break;
    case Gear::LOW:       report.report = GearReportConstants::LOW;       break;
    case Gear::LOW_2:     report.report = GearReportConstants::LOW_2;     break;
  }

  _impl->_gear_publisher.SetData(report);
}

void AutowarePublisher::SetTurnIndicators(const TurnIndicatorsStatus status)
{
  autoware_vehicle_msgs_msg_TurnIndicatorsReport report {};

  switch (status) {
    case TurnIndicatorsStatus::OFF:
      report.report = TurnIndicatorsReportConstants::DISABLE;
      break;
    case TurnIndicatorsStatus::LEFT:
      report.report = TurnIndicatorsReportConstants::ENABLE_LEFT;
      break;
    case TurnIndicatorsStatus::RIGHT:
      report.report = TurnIndicatorsReportConstants::ENABLE_RIGHT;
      break;
  }

  _impl->_turn_indicator_publisher.SetData(report);
}

void AutowarePublisher::SetHazardLights(const bool hazard_lights_enabled)
{
  autoware_vehicle_msgs_msg_HazardLightsReport report {};

  report.report =
    hazard_lights_enabled ? HazardLightsReportConstants::ENABLE
                          : HazardLightsReportConstants::DISABLE;

  _impl->_hazard_lights_publisher.SetData(report);
}

void AutowarePublisher::Publish(int32_t seconds, uint32_t nanoseconds)
{
  /**
   * @note All publishing is done at once based on assumption that all reports should be published
   * with the same frequency.
   * @sa https://github.com/tier4/AWSIM/blob/v1.3.1/docs/Components/ROS2/ROS2TopicAndServiceList/index.md
   */

  builtin_interfaces_msg_Time current_time;
  current_time.sec = seconds;
  current_time.nanosec = nanoseconds;

  _impl->_velocity_publisher.Data().header.stamp = current_time;
  _impl->_steering_publisher.Data().stamp = current_time;
  _impl->_control_mode_publisher.Data().stamp = current_time;
  _impl->_gear_publisher.Data().stamp = current_time;
  _impl->_turn_indicator_publisher.Data().stamp = current_time;
  _impl->_hazard_lights_publisher.Data().stamp = current_time;

  _impl->_velocity_publisher.Publish();
  _impl->_steering_publisher.Publish();
  _impl->_control_mode_publisher.Publish();
  _impl->_gear_publisher.Publish();
  _impl->_turn_indicator_publisher.Publish();
  _impl->_hazard_lights_publisher.Publish();
}

}  // namespace carla
}  // namespace ros2
