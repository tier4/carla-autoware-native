#pragma once

#include "ShmProtocol.h"
#include "TripleBuffer.h"
#include "SensorRegistry.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <cstdint>
#include <vector>

namespace carla {
namespace ros2 {
namespace agnocast {

class ShmWriter {
public:
  ShmWriter();
  ~ShmWriter();

  ShmWriter(const ShmWriter&) = delete;
  ShmWriter& operator=(const ShmWriter&) = delete;

  bool Init();
  void Shutdown();

  bool RegisterSensor(uint32_t sensor_id, SensorType type, uint64_t buffer_capacity);
  void UnregisterSensor(uint32_t sensor_id);

  void WriteImageData(uint32_t sensor_id,
                      int32_t sec, uint32_t nsec,
                      uint32_t height, uint32_t width,
                      const char* encoding,
                      const uint8_t* data, size_t data_size);

  void WriteLidarData(uint32_t sensor_id,
                      int32_t sec, uint32_t nsec,
                      const uint8_t* data, size_t data_size,
                      uint32_t width, uint32_t height,
                      const char* encoding);

private:
  struct SensorSegment {
    int fd = -1;
    void* mapped = nullptr;
    size_t mapped_size = 0;
    std::string shm_name;
    std::unique_ptr<TripleBufferWriter> writer;
  };

  void WriteData(SensorSegment& seg, int32_t sec, uint32_t nsec,
                 uint32_t width, uint32_t height,
                 const char* encoding,
                 const uint8_t* data, size_t data_size);

  bool CreateSegment(const std::string& name, SensorType type,
                     uint64_t buffer_capacity, SensorSegment& out);
  void DestroySegment(SensorSegment& seg);

  SensorRegistryWriter registry_;
  std::unordered_map<uint32_t, SensorSegment> segments_;
};

}  // namespace agnocast
}  // namespace ros2
}  // namespace carla
