#include "SensorRegistry.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

namespace carla {
namespace ros2 {
namespace agnocast {

SensorRegistryWriter::SensorRegistryWriter() {}
SensorRegistryWriter::~SensorRegistryWriter() { Shutdown(); }

bool SensorRegistryWriter::Init() {
  const size_t total = RegistryTotalSize();
  fd_ = shm_open(REGISTRY_SHM_NAME, O_CREAT | O_RDWR, 0666);
  if (fd_ < 0) return false;

  if (ftruncate(fd_, static_cast<off_t>(total)) != 0) {
    close(fd_); fd_ = -1; return false;
  }

  mapped_ = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (mapped_ == MAP_FAILED) {
    close(fd_); fd_ = -1; mapped_ = nullptr; return false;
  }

  std::memset(mapped_, 0, total);
  header_ = static_cast<RegistryHeader*>(mapped_);
  header_->magic = SHM_MAGIC;
  header_->version = SHM_VERSION;
  header_->sensor_count.store(0, std::memory_order_relaxed);
  header_->max_sensors = MAX_SENSORS;
  entries_ = reinterpret_cast<RegistryEntry*>(
    static_cast<uint8_t*>(mapped_) + sizeof(RegistryHeader));
  return true;
}

void SensorRegistryWriter::Shutdown() {
  if (mapped_ && mapped_ != MAP_FAILED) { munmap(mapped_, RegistryTotalSize()); mapped_ = nullptr; }
  if (fd_ >= 0) { close(fd_); shm_unlink(REGISTRY_SHM_NAME); fd_ = -1; }
  header_ = nullptr; entries_ = nullptr;
}

std::string SensorRegistryWriter::RegisterSensor(uint32_t sensor_id, SensorType type) {
  uint32_t count = header_->sensor_count.load(std::memory_order_relaxed);
  for (uint32_t i = 0; i < count; ++i) {
    if (entries_[i].sensor_id == sensor_id && entries_[i].active.load(std::memory_order_relaxed))
      return std::string(entries_[i].shm_name);
  }
  if (count >= MAX_SENSORS) return "";

  std::ostringstream oss;
  oss << "/carla_agnocast_" << static_cast<uint32_t>(type) << "_" << sensor_id;
  std::string name = oss.str();

  entries_[count].sensor_id = sensor_id;
  entries_[count].sensor_type = type;
  std::strncpy(entries_[count].shm_name, name.c_str(), 63);
  entries_[count].shm_name[63] = '\0';
  entries_[count].active.store(true, std::memory_order_release);
  header_->sensor_count.fetch_add(1, std::memory_order_release);
  return name;
}

void SensorRegistryWriter::UnregisterSensor(uint32_t sensor_id) {
  uint32_t count = header_->sensor_count.load(std::memory_order_relaxed);
  for (uint32_t i = 0; i < count; ++i) {
    if (entries_[i].sensor_id == sensor_id) {
      entries_[i].active.store(false, std::memory_order_release);
      return;
    }
  }
}

SensorRegistryReader::SensorRegistryReader() {}
SensorRegistryReader::~SensorRegistryReader() { Shutdown(); }

bool SensorRegistryReader::Init(const std::string& registry_name) {
  fd_ = shm_open(registry_name.c_str(), O_RDONLY, 0);
  if (fd_ < 0) return false;

  const size_t total = RegistryTotalSize();
  mapped_ = mmap(nullptr, total, PROT_READ, MAP_SHARED, fd_, 0);
  if (mapped_ == MAP_FAILED) {
    close(fd_); fd_ = -1; mapped_ = nullptr; return false;
  }

  header_ = static_cast<const RegistryHeader*>(mapped_);
  if (header_->magic != SHM_MAGIC || header_->version != SHM_VERSION) {
    munmap(const_cast<void*>(mapped_), total);
    close(fd_); fd_ = -1; mapped_ = nullptr; return false;
  }
  entries_ = reinterpret_cast<const RegistryEntry*>(
    static_cast<const uint8_t*>(mapped_) + sizeof(RegistryHeader));
  return true;
}

void SensorRegistryReader::Shutdown() {
  if (mapped_ && mapped_ != MAP_FAILED) { munmap(const_cast<void*>(mapped_), RegistryTotalSize()); mapped_ = nullptr; }
  if (fd_ >= 0) { close(fd_); fd_ = -1; }
  header_ = nullptr; entries_ = nullptr;
}

uint32_t SensorRegistryReader::GetSensorCount() const {
  if (!header_) return 0;
  return header_->sensor_count.load(std::memory_order_acquire);
}

SensorRegistryReader::SensorInfo SensorRegistryReader::GetSensorInfo(uint32_t index) const {
  SensorInfo info{};
  if (!entries_ || index >= GetSensorCount()) return info;
  info.sensor_id = entries_[index].sensor_id;
  info.sensor_type = entries_[index].sensor_type;
  info.shm_name = std::string(entries_[index].shm_name);
  info.active = entries_[index].active.load(std::memory_order_acquire);
  return info;
}

}  // namespace agnocast
}  // namespace ros2
}  // namespace carla
