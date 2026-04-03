// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CycloneDDSTypeRegistry.h"

#include <iostream>

// CycloneDDS idlc-generated type headers
#include "Time.h"
#include "Header.h"
#include "String.h"
#include "Float32.h"
#include "Point.h"
#include "Point32.h"
#include "Quaternion.h"
#include "Vector3.h"
#include "Pose.h"
#include "PoseStamped.h"
#include "PoseWithCovariance.h"
#include "PoseWithCovarianceStamped.h"
#include "Transform.h"
#include "TransformStamped.h"
#include "Twist.h"
#include "TwistWithCovariance.h"
#include "CameraInfo.h"
#include "Image.h"
#include "Imu.h"
#include "NavSatFix.h"
#include "NavSatStatus.h"
#include "PointCloud2.h"
#include "PointField.h"
#include "RegionOfInterest.h"
#include "Odometry.h"
#include "TFMessage.h"
#include "TF2Error.h"
#include "Clock.h"
#include "CarlaCollisionEvent.h"
#include "CarlaEgoVehicleControl.h"
#include "CarlaLineInvasion.h"
#include "Control.h"
#include "Lateral.h"
#include "Longitudinal.h"
#include "ControlModeReport.h"
#include "Engage.h"
#include "GearCommand.h"
#include "GearReport.h"
#include "HazardLightsCommand.h"
#include "HazardLightsReport.h"
#include "SteeringReport.h"
#include "TurnIndicatorsCommand.h"
#include "TurnIndicatorsReport.h"
#include "VelocityReport.h"
#include "VehicleEmergencyStamped.h"

namespace carla {
namespace ros2 {

const dds_topic_descriptor_t* CycloneDDSTypeRegistry::GetDescriptor(const std::string& type_name) {
    // Standard ROS2 types
    if (type_name == "std_msgs::msg::String")
        return &std_msgs_msg_String__desc;
    if (type_name == "std_msgs::msg::Float32")
        return &std_msgs_msg_Float32_desc;
    if (type_name == "std_msgs::msg::Header")
        return &std_msgs_msg_Header_desc;
    if (type_name == "builtin_interfaces::msg::Time")
        return &builtin_interfaces_msg_Time_desc;

    // Geometry types
    if (type_name == "geometry_msgs::msg::Point")
        return &geometry_msgs_msg_Point_desc;
    if (type_name == "geometry_msgs::msg::Point32")
        return &geometry_msgs_msg_Point32_desc;
    if (type_name == "geometry_msgs::msg::Quaternion")
        return &geometry_msgs_msg_Quaternion_desc;
    if (type_name == "geometry_msgs::msg::Vector3")
        return &geometry_msgs_msg_Vector3_desc;
    if (type_name == "geometry_msgs::msg::Pose")
        return &geometry_msgs_msg_Pose_desc;
    if (type_name == "geometry_msgs::msg::PoseStamped")
        return &geometry_msgs_msg_PoseStamped_desc;
    if (type_name == "geometry_msgs::msg::PoseWithCovariance")
        return &geometry_msgs_msg_PoseWithCovariance_desc;
    if (type_name == "geometry_msgs::msg::PoseWithCovarianceStamped")
        return &geometry_msgs_msg_PoseWithCovarianceStamped_desc;
    if (type_name == "geometry_msgs::msg::Transform")
        return &geometry_msgs_msg_Transform_desc;
    if (type_name == "geometry_msgs::msg::TransformStamped")
        return &geometry_msgs_msg_TransformStamped_desc;
    if (type_name == "geometry_msgs::msg::Twist")
        return &geometry_msgs_msg_Twist_desc;
    if (type_name == "geometry_msgs::msg::TwistWithCovariance")
        return &geometry_msgs_msg_TwistWithCovariance_desc;

    // Sensor types
    if (type_name == "sensor_msgs::msg::CameraInfo")
        return &sensor_msgs_msg_CameraInfo_desc;
    if (type_name == "sensor_msgs::msg::Image")
        return &sensor_msgs_msg_Image_desc;
    if (type_name == "sensor_msgs::msg::Imu")
        return &sensor_msgs_msg_Imu_desc;
    if (type_name == "sensor_msgs::msg::NavSatFix")
        return &sensor_msgs_msg_NavSatFix_desc;
    if (type_name == "sensor_msgs::msg::NavSatStatus")
        return &sensor_msgs_msg_NavSatStatus_desc;
    if (type_name == "sensor_msgs::msg::PointCloud2")
        return &sensor_msgs_msg_PointCloud2_desc;
    if (type_name == "sensor_msgs::msg::PointField")
        return &sensor_msgs_msg_PointField_desc;
    if (type_name == "sensor_msgs::msg::RegionOfInterest")
        return &sensor_msgs_msg_RegionOfInterest_desc;

    // Navigation types
    if (type_name == "nav_msgs::msg::Odometry")
        return &nav_msgs_msg_Odometry_desc;

    // TF2 types
    if (type_name == "tf2_msgs::msg::TFMessage")
        return &tf2_msgs_msg_TFMessage_desc;
    if (type_name == "tf2_msgs::msg::TF2Error")
        return &tf2_msgs_msg_TF2Error_desc;

    // Clock
    if (type_name == "rosgraph_msgs::msg::Clock")
        return &rosgraph_msgs_msg_Clock_desc;
    // Alias used in some existing code
    if (type_name == "rosgraph::msg::Clock")
        return &rosgraph_msgs_msg_Clock_desc;

    // CARLA custom types
    if (type_name == "carla_msgs::msg::CarlaCollisionEvent")
        return &carla_msgs_msg_CarlaCollisionEvent_desc;
    if (type_name == "carla_msgs::msg::CarlaEgoVehicleControl")
        return &carla_msgs_msg_CarlaEgoVehicleControl_desc;
    if (type_name == "carla_msgs::msg::CarlaLineInvasion")
        return &carla_msgs_msg_CarlaLineInvasion_desc;
    // Alias
    if (type_name == "carla_msgs::msg::LaneInvasionEvent")
        return &carla_msgs_msg_CarlaLineInvasion_desc;

    // Autoware control types
    if (type_name == "autoware_control_msgs::msg::Control")
        return &autoware_control_msgs_msg_Control_desc;
    if (type_name == "autoware_control_msgs::msg::Lateral")
        return &autoware_control_msgs_msg_Lateral_desc;
    if (type_name == "autoware_control_msgs::msg::Longitudinal")
        return &autoware_control_msgs_msg_Longitudinal_desc;

    // Autoware vehicle types
    if (type_name == "autoware_vehicle_msgs::msg::ControlModeReport")
        return &autoware_vehicle_msgs_msg_ControlModeReport_desc;
    if (type_name == "autoware_vehicle_msgs::msg::Engage")
        return &autoware_vehicle_msgs_msg_Engage_desc;
    if (type_name == "autoware_vehicle_msgs::msg::GearCommand")
        return &autoware_vehicle_msgs_msg_GearCommand_desc;
    if (type_name == "autoware_vehicle_msgs::msg::GearReport")
        return &autoware_vehicle_msgs_msg_GearReport_desc;
    if (type_name == "autoware_vehicle_msgs::msg::HazardLightsCommand")
        return &autoware_vehicle_msgs_msg_HazardLightsCommand_desc;
    if (type_name == "autoware_vehicle_msgs::msg::HazardLightsReport")
        return &autoware_vehicle_msgs_msg_HazardLightsReport_desc;
    if (type_name == "autoware_vehicle_msgs::msg::SteeringReport")
        return &autoware_vehicle_msgs_msg_SteeringReport_desc;
    if (type_name == "autoware_vehicle_msgs::msg::TurnIndicatorsCommand")
        return &autoware_vehicle_msgs_msg_TurnIndicatorsCommand_desc;
    if (type_name == "autoware_vehicle_msgs::msg::TurnIndicatorsReport")
        return &autoware_vehicle_msgs_msg_TurnIndicatorsReport_desc;
    if (type_name == "autoware_vehicle_msgs::msg::VelocityReport")
        return &autoware_vehicle_msgs_msg_VelocityReport_desc;

    // Tier4 types
    if (type_name == "tier4_vehicle_msgs::msg::VehicleEmergencyStamped")
        return &tier4_vehicle_msgs_msg_VehicleEmergencyStamped_desc;

    std::cerr << "CycloneDDSTypeRegistry: Unknown type: " << type_name << std::endl;
    return nullptr;
}

} // namespace ros2
} // namespace carla
