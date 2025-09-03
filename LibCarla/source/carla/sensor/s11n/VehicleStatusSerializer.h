#pragma once

#include "carla/Buffer.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/SensorData.h"

namespace carla {
namespace sensor {
namespace s11n {

class VehicleStatusSerializer {
public:

  struct Data {
    double timestamp;
    float speed_mps;
    float vel_x_mps, vel_y_mps, vel_z_mps; // local
    float angVel_x_mps, angVel_y_mps, angVel_z_mps; // local
    float rotr_pitch, rotr_yaw, rotr_roll; // local
    float steer;
    int32_t gear;
    uint8_t turn_mask;
    uint8_t control_flags;

    MSGPACK_DEFINE_ARRAY(timestamp,
                         speed_mps,
                         vel_x_mps, vel_y_mps, vel_z_mps,
                         angVel_x_mps, angVel_y_mps, angVel_z_mps,
                         rotr_pitch, rotr_yaw, rotr_roll,
                         steer,
                         gear,
                         turn_mask,
                         control_flags)
  };

  constexpr static auto header_offset = 0u;
  
  static Data DeserializeRawData(const RawData &message) {
    return MsgPack::UnPack<Data>(message.begin(), message.size());
  }

  // Existing version for direct calls
  template <typename SensorT>
  static Buffer Serialize(
      const SensorT &,
      double timestamp,
      float speed_mps,
      float vel_x_mps,
      float vel_y_mps,
      float vel_z_mps,
      float angVel_x_mps,
      float angVel_y_mps,
      float angVel_z_mps,
      float rotr_pitch,
      float rotr_yaw,
      float rotr_roll,
      float steer,
      int32_t gear,
      uint8_t turn_mask,
      uint8_t control_flags) {
    return MsgPack::Pack(Data{
        timestamp,
        speed_mps,
        vel_x_mps, vel_y_mps, vel_z_mps,
        angVel_x_mps, angVel_y_mps, angVel_z_mps,
        rotr_pitch, rotr_yaw, rotr_roll,
        steer,
        gear,
        turn_mask,
        control_flags});
  }

  // New overload for CompositeSerializer / Unreal
  template <typename SensorT>
  static Buffer Serialize(const SensorT &sensor, const Buffer &buffer) {
    // Convert Unreal’s Packed struct to MsgPack
    struct Packed {
      double timestamp;
      float speed_mps;
      float vel_x_mps, vel_y_mps, vel_z_mps; // local
      float angVel_x_mps, angVel_y_mps, angVel_z_mps; // local
      float rotr_pitch, rotr_yaw, rotr_roll; // local
      float steer;
      int32_t gear;
      uint8_t turn_mask;
      uint8_t control_flags;
      uint8_t _pad0;
      uint8_t _pad1;
    } PACKED;

    if (buffer.size() != sizeof(Packed)) {
      throw std::runtime_error("Invalid buffer size for VehicleStatusSensor");
    }

    Packed msg;
    std::memcpy(&msg, buffer.data(), sizeof(Packed));

    return MsgPack::Pack(Data{
        msg.timestamp,
        msg.speed_mps,
        msg.vel_x_mps, msg.vel_y_mps, msg.vel_z_mps,
        msg.angVel_x_mps, msg.angVel_y_mps, msg.angVel_z_mps,
        msg.rotr_pitch, msg.rotr_yaw, msg.rotr_roll,
        msg.steer,
        msg.gear,
        msg.turn_mask,
        msg.control_flags});
  }

  static SharedPtr<SensorData> Deserialize(RawData &&data);
};

} // namespace s11n
} // namespace sensor
} // namespace carla
