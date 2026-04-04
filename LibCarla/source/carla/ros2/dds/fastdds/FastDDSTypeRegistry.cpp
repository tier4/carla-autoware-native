// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "FastDDSTypeRegistry.h"

#include <iostream>
#include <stdexcept>
#include <string>

// Normalize DDS wire format type names to ROS2 names.
// FastDDS PubSubType::getName() returns "pkg::msg::dds_::Type_" but
// the registry uses ROS2 names "pkg::msg::Type".
static std::string NormalizeDDSTypeName(const std::string& name) {
    const std::string dds_infix = "::msg::dds_::";
    auto pos = name.find(dds_infix);
    if (pos == std::string::npos) return name;

    std::string pkg = name.substr(0, pos);
    std::string type = name.substr(pos + dds_infix.size());
    if (!type.empty() && type.back() == '_') type.pop_back();
    return pkg + "::msg::" + type;
}

// All PubSubTypes headers
#include "carla/ros2/dds/fastdds/types/CameraInfoPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/CarlaCollisionEventPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/CarlaEgoVehicleControlPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/CarlaLineInvasionPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/ClockPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/ControlModeReportPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/ControlPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/EngagePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/Float32PubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/GearCommandPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/GearReportPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/HazardLightsCommandPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/HazardLightsReportPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/HeaderPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/ImagePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/ImuPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/LateralPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/LongitudinalPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/NavSatFixPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/NavSatStatusPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/OdometryPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/Point32PubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PointCloud2PubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PointFieldPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PointPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PosePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PoseStampedPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PoseWithCovariancePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/PoseWithCovarianceStampedPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/QuaternionPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/RegionOfInterestPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/SteeringReportPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/StringPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TF2ErrorPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TFMessagePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TimePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TransformPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TransformStampedPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TurnIndicatorsCommandPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TurnIndicatorsReportPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TwistPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/TwistWithCovariancePubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/Vector3PubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/VehicleEmergencyStampedPubSubTypes.h"
#include "carla/ros2/dds/fastdds/types/VelocityReportPubSubTypes.h"

namespace carla {
namespace ros2 {

efd::TypeSupport FastDDSTypeRegistry::Create(const std::string& type_name) {
    // Normalize DDS wire format names (from PubSubType::getName()) to ROS2 names
    const std::string normalized = NormalizeDDSTypeName(type_name);

    // sensor_msgs
    if (normalized == "sensor_msgs::msg::CameraInfo")
        return efd::TypeSupport(new sensor_msgs::msg::CameraInfoPubSubType());
    if (normalized == "sensor_msgs::msg::Image")
        return efd::TypeSupport(new sensor_msgs::msg::ImagePubSubType());
    if (normalized == "sensor_msgs::msg::Imu")
        return efd::TypeSupport(new sensor_msgs::msg::ImuPubSubType());
    if (normalized == "sensor_msgs::msg::NavSatFix")
        return efd::TypeSupport(new sensor_msgs::msg::NavSatFixPubSubType());
    if (normalized == "sensor_msgs::msg::NavSatStatus")
        return efd::TypeSupport(new sensor_msgs::msg::NavSatStatusPubSubType());
    if (normalized == "sensor_msgs::msg::PointCloud2")
        return efd::TypeSupport(new sensor_msgs::msg::PointCloud2PubSubType());
    if (normalized == "sensor_msgs::msg::PointField")
        return efd::TypeSupport(new sensor_msgs::msg::PointFieldPubSubType());
    if (normalized == "sensor_msgs::msg::RegionOfInterest")
        return efd::TypeSupport(new sensor_msgs::msg::RegionOfInterestPubSubType());

    // std_msgs
    if (normalized == "std_msgs::msg::Float32")
        return efd::TypeSupport(new std_msgs::msg::Float32PubSubType());
    if (normalized == "std_msgs::msg::Header")
        return efd::TypeSupport(new std_msgs::msg::HeaderPubSubType());
    if (normalized == "std_msgs::msg::String")
        return efd::TypeSupport(new std_msgs::msg::StringPubSubType());

    // geometry_msgs
    if (normalized == "geometry_msgs::msg::Point")
        return efd::TypeSupport(new geometry_msgs::msg::PointPubSubType());
    if (normalized == "geometry_msgs::msg::Point32")
        return efd::TypeSupport(new geometry_msgs::msg::Point32PubSubType());
    if (normalized == "geometry_msgs::msg::Pose")
        return efd::TypeSupport(new geometry_msgs::msg::PosePubSubType());
    if (normalized == "geometry_msgs::msg::PoseStamped")
        return efd::TypeSupport(new geometry_msgs::msg::PoseStampedPubSubType());
    if (normalized == "geometry_msgs::msg::PoseWithCovariance")
        return efd::TypeSupport(new geometry_msgs::msg::PoseWithCovariancePubSubType());
    if (normalized == "geometry_msgs::msg::PoseWithCovarianceStamped")
        return efd::TypeSupport(new geometry_msgs::msg::PoseWithCovarianceStampedPubSubType());
    if (normalized == "geometry_msgs::msg::Quaternion")
        return efd::TypeSupport(new geometry_msgs::msg::QuaternionPubSubType());
    if (normalized == "geometry_msgs::msg::Transform")
        return efd::TypeSupport(new geometry_msgs::msg::TransformPubSubType());
    if (normalized == "geometry_msgs::msg::TransformStamped")
        return efd::TypeSupport(new geometry_msgs::msg::TransformStampedPubSubType());
    if (normalized == "geometry_msgs::msg::Twist")
        return efd::TypeSupport(new geometry_msgs::msg::TwistPubSubType());
    if (normalized == "geometry_msgs::msg::TwistWithCovariance")
        return efd::TypeSupport(new geometry_msgs::msg::TwistWithCovariancePubSubType());
    if (normalized == "geometry_msgs::msg::Vector3")
        return efd::TypeSupport(new geometry_msgs::msg::Vector3PubSubType());

    // nav_msgs
    if (normalized == "nav_msgs::msg::Odometry")
        return efd::TypeSupport(new nav_msgs::msg::OdometryPubSubType());

    // rosgraph_msgs (namespace is 'rosgraph' in generated code)
    if (normalized == "rosgraph_msgs::msg::Clock" || type_name == "rosgraph::msg::Clock")
        return efd::TypeSupport(new rosgraph::msg::ClockPubSubType());

    // builtin_interfaces
    if (normalized == "builtin_interfaces::msg::Time")
        return efd::TypeSupport(new builtin_interfaces::msg::TimePubSubType());

    // tf2_msgs
    if (normalized == "tf2_msgs::msg::TF2Error")
        return efd::TypeSupport(new tf2_msgs::msg::TF2ErrorPubSubType());
    if (normalized == "tf2_msgs::msg::TFMessage")
        return efd::TypeSupport(new tf2_msgs::msg::TFMessagePubSubType());

    // carla_msgs
    if (normalized == "carla_msgs::msg::CarlaCollisionEvent")
        return efd::TypeSupport(new carla_msgs::msg::CarlaCollisionEventPubSubType());
    if (normalized == "carla_msgs::msg::CarlaEgoVehicleControl")
        return efd::TypeSupport(new carla_msgs::msg::CarlaEgoVehicleControlPubSubType());
    if (normalized == "carla_msgs::msg::LaneInvasionEvent")
        return efd::TypeSupport(new carla_msgs::msg::LaneInvasionEventPubSubType());

    // autoware_control_msgs
    if (normalized == "autoware_control_msgs::msg::Control")
        return efd::TypeSupport(new autoware_control_msgs::msg::ControlPubSubType());
    if (normalized == "autoware_control_msgs::msg::Lateral")
        return efd::TypeSupport(new autoware_control_msgs::msg::LateralPubSubType());
    if (normalized == "autoware_control_msgs::msg::Longitudinal")
        return efd::TypeSupport(new autoware_control_msgs::msg::LongitudinalPubSubType());

    // autoware_vehicle_msgs
    if (normalized == "autoware_vehicle_msgs::msg::ControlModeReport")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::ControlModeReportPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::Engage")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::EngagePubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::GearCommand")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::GearCommandPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::GearReport")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::GearReportPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::HazardLightsCommand")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::HazardLightsCommandPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::HazardLightsReport")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::HazardLightsReportPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::SteeringReport")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::SteeringReportPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::TurnIndicatorsCommand")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::TurnIndicatorsCommandPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::TurnIndicatorsReport")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::TurnIndicatorsReportPubSubType());
    if (normalized == "autoware_vehicle_msgs::msg::VelocityReport")
        return efd::TypeSupport(new autoware_vehicle_msgs::msg::VelocityReportPubSubType());

    // tier4_vehicle_msgs
    if (normalized == "tier4_vehicle_msgs::msg::VehicleEmergencyStamped")
        return efd::TypeSupport(new tier4_vehicle_msgs::msg::VehicleEmergencyStampedPubSubType());

    std::cerr << "FastDDSTypeRegistry::Create: unknown type '" << type_name
              << "' (normalized: '" << normalized << "')" << std::endl;
    return efd::TypeSupport(nullptr);
}

} // namespace ros2
} // namespace carla
