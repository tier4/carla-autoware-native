// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "AutowarePublisher.h"
#include <string>
#include <iostream>

#include "dds/dds.h"
#include "VelocityReport.h"
#include "SteeringReport.h"
#include "ControlModeReport.h"
#include "GearReport.h"
#include "TurnIndicatorsReport.h"
#include "HazardLightsReport.h"

#include "carla/ros2/AutowareSteeringCompensation.h"

namespace carla {
namespace ros2 {

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
// Per-report sub-publisher: direct CycloneDDS C API
// ---------------------------------------------------------------------------
struct ReportWriter {
  dds_entity_t participant { 0 };
  dds_entity_t topic { 0 };
  dds_entity_t writer { 0 };

  bool Init(DomainId domain_id, const dds_topic_descriptor_t* desc, const char* topic_name, const char* human_name) {
    participant = dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr);
    if (participant < 0) {
      std::cerr << "CycloneDDS: Failed to create DomainParticipant for " << human_name << std::endl;
      return false;
    }

    topic = dds_create_topic(participant, desc, topic_name, nullptr, nullptr);
    if (topic < 0) {
      std::cerr << "CycloneDDS: Failed to create Topic for " << human_name << ": " << dds_strretcode(-topic) << std::endl;
      dds_delete(participant);
      participant = 0;
      return false;
    }

    // RELIABLE + VOLATILE QoS
    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

    writer = dds_create_writer(participant, topic, qos, nullptr);
    dds_delete_qos(qos);
    if (writer < 0) {
      std::cerr << "CycloneDDS: Failed to create DataWriter for " << human_name << std::endl;
      dds_delete(participant);
      participant = 0;
      return false;
    }
    return true;
  }

  ~ReportWriter() {
    // Note: intentionally not calling dds_delete in destructor;
    // cleanup is handled by the parent Implementation destructor.
  }
};

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------
class AutowarePublisher::Implementation
{
public:
  ReportWriter _velocity_writer{};
  ReportWriter _steering_writer{};
  ReportWriter _control_mode_writer{};
  ReportWriter _gear_writer{};
  ReportWriter _turn_indicator_writer{};
  ReportWriter _hazard_lights_writer{};

  autoware_vehicle_msgs_msg_VelocityReport       _velocity_msg {};
  autoware_vehicle_msgs_msg_SteeringReport        _steering_msg {};
  autoware_vehicle_msgs_msg_ControlModeReport     _control_mode_msg {};
  autoware_vehicle_msgs_msg_GearReport            _gear_msg {};
  autoware_vehicle_msgs_msg_TurnIndicatorsReport  _turn_indicator_msg {};
  autoware_vehicle_msgs_msg_HazardLightsReport    _hazard_lights_msg {};

  std::string _velocity_frame_id_store;

  void Cleanup() {
    auto cleanup = [](ReportWriter& w) {
      if (w.participant > 0) {
        dds_delete(w.participant);
        w.participant = 0;
      }
    };
    cleanup(_velocity_writer);
    cleanup(_steering_writer);
    cleanup(_control_mode_writer);
    cleanup(_gear_writer);
    cleanup(_turn_indicator_writer);
    cleanup(_hazard_lights_writer);
  }

  ~Implementation() {
    Cleanup();
  }
};

AutowarePublisher::AutowarePublisher([[maybe_unused]] void* vehicle, const DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  _impl->_velocity_writer.Init(domain_id,
    &autoware_vehicle_msgs_msg_VelocityReport_desc,
    "rt/vehicle/status/velocity_status", "Autoware velocity report");

  _impl->_steering_writer.Init(domain_id,
    &autoware_vehicle_msgs_msg_SteeringReport_desc,
    "rt/vehicle/status/steering_status", "Autoware steering report");

  _impl->_control_mode_writer.Init(domain_id,
    &autoware_vehicle_msgs_msg_ControlModeReport_desc,
    "rt/vehicle/status/control_mode", "Autoware control mode report");

  _impl->_gear_writer.Init(domain_id,
    &autoware_vehicle_msgs_msg_GearReport_desc,
    "rt/vehicle/status/gear_status", "Autoware gear report");

  _impl->_turn_indicator_writer.Init(domain_id,
    &autoware_vehicle_msgs_msg_TurnIndicatorsReport_desc,
    "rt/vehicle/status/turn_indicators_status", "Autoware turn indicators report");

  _impl->_hazard_lights_writer.Init(domain_id,
    &autoware_vehicle_msgs_msg_HazardLightsReport_desc,
    "rt/vehicle/status/hazard_lights_status", "Autoware hazard lights report");
}

AutowarePublisher::~AutowarePublisher() = default;

void AutowarePublisher::SetVelocity(float longitudinal_velocity, float lateral_velocity, float heading_rate)
{
  _impl->_velocity_frame_id_store = "base_link";

  std_msgs_msg_Header header {};
  header.frame_id = const_cast<char*>(_impl->_velocity_frame_id_store.c_str());

  autoware_vehicle_msgs_msg_VelocityReport report {};
  report.header = header;
  report.longitudinal_velocity = longitudinal_velocity;
  report.lateral_velocity = lateral_velocity;
  report.heading_rate = heading_rate;

  _impl->_velocity_msg = report;
}

void AutowarePublisher::SetSteering(float steering_tire_angle)
{
  autoware_vehicle_msgs_msg_SteeringReport report {};

  report.steering_tire_angle = autoware_steering_compensation::GetSteeringOutput(steering_tire_angle);

  _impl->_steering_msg = report;
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

  _impl->_control_mode_msg = report;
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

  _impl->_gear_msg = report;
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

  _impl->_turn_indicator_msg = report;
}

void AutowarePublisher::SetHazardLights(const bool hazard_lights_enabled)
{
  autoware_vehicle_msgs_msg_HazardLightsReport report {};

  report.report =
    hazard_lights_enabled ? HazardLightsReportConstants::ENABLE
                          : HazardLightsReportConstants::DISABLE;

  _impl->_hazard_lights_msg = report;
}

void AutowarePublisher::Publish(int32_t seconds, uint32_t nanoseconds)
{
  builtin_interfaces_msg_Time current_time;
  current_time.sec = seconds;
  current_time.nanosec = nanoseconds;

  _impl->_velocity_msg.header.stamp = current_time;
  _impl->_steering_msg.stamp = current_time;
  _impl->_control_mode_msg.stamp = current_time;
  _impl->_gear_msg.stamp = current_time;
  _impl->_turn_indicator_msg.stamp = current_time;
  _impl->_hazard_lights_msg.stamp = current_time;

  dds_write(_impl->_velocity_writer.writer, &_impl->_velocity_msg);
  dds_write(_impl->_steering_writer.writer, &_impl->_steering_msg);
  dds_write(_impl->_control_mode_writer.writer, &_impl->_control_mode_msg);
  dds_write(_impl->_gear_writer.writer, &_impl->_gear_msg);
  dds_write(_impl->_turn_indicator_writer.writer, &_impl->_turn_indicator_msg);
  dds_write(_impl->_hazard_lights_writer.writer, &_impl->_hazard_lights_msg);
}

}  // namespace ros2
}  // namespace carla
