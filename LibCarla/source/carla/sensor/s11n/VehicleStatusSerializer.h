#pragma once

#include "carla/Buffer.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/SensorData.h"

namespace carla {
namespace sensor {
namespace s11n {

#pragma pack(push, 1)
struct VehicleStatusData 
{
  double timestamp;
  float speed_mps;
  float vel_x_mps, vel_y_mps, vel_z_mps; // local
  float angVel_x_mps, angVel_y_mps, angVel_z_mps; // local
  float rot_pitch, rot_yaw, rot_roll; // relative
  float steer;
  int32_t gear;
  uint8_t turn_mask;
  uint8_t control_flags;

  MSGPACK_DEFINE_ARRAY(timestamp,
                        speed_mps,
                        vel_x_mps, vel_y_mps, vel_z_mps,
                        angVel_x_mps, angVel_y_mps, angVel_z_mps,
                        rot_pitch, rot_yaw, rot_roll,
                        steer,
                        gear,
                        turn_mask,
                        control_flags)
};
#pragma pack(pop)

class VehicleStatusSerializer {
public:
  constexpr static auto header_offset = 0u;
  
  static VehicleStatusData DeserializeRawData(const RawData &message) {
    return MsgPack::UnPack<VehicleStatusData>(message.begin(), message.size());
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
      float rot_pitch,
      float rot_yaw,
      float rot_roll,
      float steer,
      int32_t gear,
      uint8_t turn_mask,
      uint8_t control_flags) {
    return MsgPack::Pack(VehicleStatusData{
        timestamp,
        speed_mps,
        vel_x_mps, vel_y_mps, vel_z_mps,
        angVel_x_mps, angVel_y_mps, angVel_z_mps,
        rot_pitch, rot_yaw, rot_roll,
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
      float rot_pitch, rot_yaw, rot_roll; // local
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

    return MsgPack::Pack(VehicleStatusData{
        msg.timestamp,
        msg.speed_mps,
        msg.vel_x_mps, msg.vel_y_mps, msg.vel_z_mps,
        msg.angVel_x_mps, msg.angVel_y_mps, msg.angVel_z_mps,
        msg.rot_pitch, msg.rot_yaw, msg.rot_roll,
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
