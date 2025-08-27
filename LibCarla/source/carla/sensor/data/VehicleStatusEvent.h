#pragma once

#include "carla/sensor/SensorData.h"
#include "carla/sensor/s11n/VehicleStatusSerializer.h"

namespace carla {
namespace sensor {
namespace data {

  class VehicleStatusEvent : public SensorData {
    using Super = SensorData;
  protected:
    using Serializer = s11n::VehicleStatusSerializer;
    friend Serializer;

    explicit VehicleStatusEvent(const RawData &data)
      : Super(data),
        _parsed(Serializer::DeserializeRawData(data)) {}

  public:

    double GetTimestamp() const { return _parsed.timestamp; }

    float GetSpeed() const { return _parsed.speed_mps; }
    float GetVelX() const { return _parsed.vel_x_mps; } // local
    float GetVelY() const { return _parsed.vel_y_mps; } // local
    float GetVelZ() const { return _parsed.vel_z_mps; } // local

    float GetAngVelX() const { return _parsed.angVel_x_mps; } // local
    float GetAngVelY() const { return _parsed.angVel_y_mps; } // local
    float GetAngVelZ() const { return _parsed.angVel_z_mps; } // local

    float GetRotrPitch() const { return _parsed.rotr_pitch; } // local
    float GetRotrYaw() const { return _parsed.rotr_yaw; } // local
    float GetRotrRoll() const { return _parsed.rotr_roll; } // local
    

    float GetSteer() const { return _parsed.steer; }
    int32_t GetGear() const { return _parsed.gear; }
    uint8_t GetTurnMask() const { return _parsed.turn_mask; }
    uint8_t GetControlFlags() const { return _parsed.control_flags; }

  private:
    Serializer::Data _parsed;
  };

} // namespace data
} // namespace sensor
} // namespace carla
