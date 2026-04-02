#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

namespace carla {
namespace ros2 {
namespace agnocast {

static constexpr uint32_t SHM_MAGIC = 0xA6C0CA57;  // "AGNOCAST"
static constexpr uint32_t SHM_VERSION = 1;
static constexpr uint32_t NUM_SLOTS = 3;
static constexpr uint32_t MAX_SENSORS = 64;
static constexpr size_t CACHE_LINE_SIZE = 64;
static constexpr char REGISTRY_SHM_NAME[] = "/carla_agnocast_registry";

enum class SensorType : uint32_t {
  RGBCamera = 0,
  DepthCamera = 1,
  SemanticSegmentationCamera = 2,
  InstanceSegmentationCamera = 3,
  NormalsCamera = 4,
  OpticalFlowCamera = 5,
  DVSCamera = 6,
  RayCastLidar = 7,
  RayCastSemanticLidar = 8,
};

struct alignas(CACHE_LINE_SIZE) SlotMetadata {
  uint64_t frame_id;
  int32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint32_t width;
  uint32_t height;
  uint64_t data_size;
  char encoding[16];
  uint8_t padding[4];
};
static_assert(sizeof(SlotMetadata) == CACHE_LINE_SIZE, "SlotMetadata must be cache-line sized");

struct alignas(CACHE_LINE_SIZE) ShmHeader {
  uint32_t magic;
  uint32_t version;
  SensorType sensor_type;
  uint32_t reserved0;

  std::atomic<uint64_t> write_frame_id;
  std::atomic<uint32_t> latest_slot;
  uint32_t reserved1;

  uint64_t buffer_capacity;
  uint64_t total_size;

  uint8_t padding[CACHE_LINE_SIZE - 48];
};
static_assert(sizeof(ShmHeader) == CACHE_LINE_SIZE, "ShmHeader must be cache-line sized");

inline size_t SlotOffset(uint32_t slot_index, uint64_t buffer_capacity) {
  return sizeof(ShmHeader) + slot_index * (sizeof(SlotMetadata) + buffer_capacity);
}

inline SlotMetadata* GetSlotMetadata(void* base, uint32_t slot_index, uint64_t buffer_capacity) {
  return reinterpret_cast<SlotMetadata*>(
    static_cast<uint8_t*>(base) + SlotOffset(slot_index, buffer_capacity));
}

inline const SlotMetadata* GetSlotMetadata(const void* base, uint32_t slot_index, uint64_t buffer_capacity) {
  return reinterpret_cast<const SlotMetadata*>(
    static_cast<const uint8_t*>(base) + SlotOffset(slot_index, buffer_capacity));
}

inline uint8_t* GetSlotData(void* base, uint32_t slot_index, uint64_t buffer_capacity) {
  return static_cast<uint8_t*>(base) + SlotOffset(slot_index, buffer_capacity) + sizeof(SlotMetadata);
}

inline const uint8_t* GetSlotData(const void* base, uint32_t slot_index, uint64_t buffer_capacity) {
  return static_cast<const uint8_t*>(base) + SlotOffset(slot_index, buffer_capacity) + sizeof(SlotMetadata);
}

inline size_t CalculateTotalSize(uint64_t buffer_capacity) {
  return sizeof(ShmHeader) + NUM_SLOTS * (sizeof(SlotMetadata) + buffer_capacity);
}

struct RegistryEntry {
  uint32_t sensor_id;
  SensorType sensor_type;
  char shm_name[64];
  std::atomic<bool> active;
  // Total so far: 4 + 4 + 64 + 1 = 73 bytes; pad to 128 bytes (2 cache lines)
  uint8_t padding[128 - sizeof(uint32_t) - sizeof(SensorType) - 64 - sizeof(std::atomic<bool>)];
};
static_assert(sizeof(RegistryEntry) == 128, "RegistryEntry must be 128 bytes");

struct alignas(CACHE_LINE_SIZE) RegistryHeader {
  uint32_t magic;
  uint32_t version;
  std::atomic<uint32_t> sensor_count;
  uint32_t max_sensors;
  uint8_t padding[CACHE_LINE_SIZE - 16];
};
static_assert(sizeof(RegistryHeader) == CACHE_LINE_SIZE, "RegistryHeader must be cache-line sized");

inline size_t RegistryTotalSize() {
  return sizeof(RegistryHeader) + MAX_SENSORS * sizeof(RegistryEntry);
}

}  // namespace agnocast
}  // namespace ros2
}  // namespace carla
