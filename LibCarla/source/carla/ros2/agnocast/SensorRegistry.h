#pragma once

#include "ShmProtocol.h"
#include <string>
#include <cstdint>

namespace carla {
namespace ros2 {
namespace agnocast {

class SensorRegistryWriter {
public:
  SensorRegistryWriter();
  ~SensorRegistryWriter();

  SensorRegistryWriter(const SensorRegistryWriter&) = delete;
  SensorRegistryWriter& operator=(const SensorRegistryWriter&) = delete;

  bool Init();
  void Shutdown();

  std::string RegisterSensor(uint32_t sensor_id, SensorType type);
  void UnregisterSensor(uint32_t sensor_id);

private:
  int fd_ = -1;
  void* mapped_ = nullptr;
  RegistryHeader* header_ = nullptr;
  RegistryEntry* entries_ = nullptr;
};

class SensorRegistryReader {
public:
  SensorRegistryReader();
  ~SensorRegistryReader();

  SensorRegistryReader(const SensorRegistryReader&) = delete;
  SensorRegistryReader& operator=(const SensorRegistryReader&) = delete;

  bool Init(const std::string& registry_name = REGISTRY_SHM_NAME);
  void Shutdown();
  bool IsAvailable() const { return mapped_ != nullptr; }

  uint32_t GetSensorCount() const;

  struct SensorInfo {
    uint32_t sensor_id;
    SensorType sensor_type;
    std::string shm_name;
    bool active;
  };

  SensorInfo GetSensorInfo(uint32_t index) const;

private:
  int fd_ = -1;
  void* mapped_ = nullptr;
  const RegistryHeader* header_ = nullptr;
  const RegistryEntry* entries_ = nullptr;
};

}  // namespace agnocast
}  // namespace ros2
}  // namespace carla
