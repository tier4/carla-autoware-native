#pragma once

#include <cstdint>

namespace carla {
namespace sensor {
namespace data {

  #pragma pack(push, 1)
  struct VehicleStatusMessageRaw
  {
    public: 
      double timestamp;
      float speed_mps;
      float vel_x_mps, vel_y_mps, vel_z_mps; // local
      float angVel_x_mps, angVel_y_mps, angVel_z_mps; // local
      float rotr_pitch, rotr_yaw, rotr_roll; // local
      float steer;
      int32_t gear;
      uint8_t turn_mask;
      uint8_t control_flags;
  };
  #pragma pack(pop)

} // namespace data
} // namespace sensor
} // namespace carla
