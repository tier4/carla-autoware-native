#include "carla/sensor/data/VehicleStatusEvent.h"
#include "VehicleStatusSerializer.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> VehicleStatusSerializer::Deserialize(RawData &&data) {
    return SharedPtr<SensorData>(new data::VehicleStatusEvent(std::move(data)));
  }
} // namespace s11n
} // namespace sensor
} // namespace carla
