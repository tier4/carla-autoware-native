#include "ShmWriter.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

namespace carla {
namespace ros2 {
namespace agnocast {

ShmWriter::ShmWriter() {}
ShmWriter::~ShmWriter() { Shutdown(); }

bool ShmWriter::Init() { return registry_.Init(); }

void ShmWriter::Shutdown() {
  for (auto& [id, seg] : segments_) {
    registry_.UnregisterSensor(id);
    DestroySegment(seg);
  }
  segments_.clear();
  registry_.Shutdown();
}

bool ShmWriter::RegisterSensor(uint32_t sensor_id, SensorType type, uint64_t buffer_capacity) {
  if (segments_.count(sensor_id)) return true;
  std::string name = registry_.RegisterSensor(sensor_id, type);
  if (name.empty()) return false;

  SensorSegment seg;
  if (!CreateSegment(name, type, buffer_capacity, seg)) {
    registry_.UnregisterSensor(sensor_id);
    return false;
  }
  segments_.emplace(sensor_id, std::move(seg));
  return true;
}

void ShmWriter::UnregisterSensor(uint32_t sensor_id) {
  auto it = segments_.find(sensor_id);
  if (it == segments_.end()) return;
  registry_.UnregisterSensor(sensor_id);
  DestroySegment(it->second);
  segments_.erase(it);
}

bool ShmWriter::CreateSegment(const std::string& name, SensorType type,
                              uint64_t buffer_capacity, SensorSegment& out) {
  size_t total = CalculateTotalSize(buffer_capacity);
  int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd < 0) return false;
  if (ftruncate(fd, static_cast<off_t>(total)) != 0) {
    close(fd); shm_unlink(name.c_str()); return false;
  }
  void* mapped = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (mapped == MAP_FAILED) {
    close(fd); shm_unlink(name.c_str()); return false;
  }
  std::memset(mapped, 0, total);
  auto* header = static_cast<ShmHeader*>(mapped);
  header->magic = SHM_MAGIC;
  header->version = SHM_VERSION;
  header->sensor_type = type;
  header->buffer_capacity = buffer_capacity;
  header->total_size = total;
  header->latest_slot.store(0, std::memory_order_relaxed);
  header->write_frame_id.store(0, std::memory_order_relaxed);

  out.fd = fd;
  out.mapped = mapped;
  out.mapped_size = total;
  out.shm_name = name;
  out.writer = std::make_unique<TripleBufferWriter>(mapped);
  return true;
}

void ShmWriter::DestroySegment(SensorSegment& seg) {
  if (seg.mapped && seg.mapped != MAP_FAILED) munmap(seg.mapped, seg.mapped_size);
  if (seg.fd >= 0) { close(seg.fd); shm_unlink(seg.shm_name.c_str()); }
  seg.mapped = nullptr; seg.fd = -1;
}

void ShmWriter::WriteData(SensorSegment& seg, int32_t sec, uint32_t nsec,
                          uint32_t width, uint32_t height,
                          const char* encoding,
                          const uint8_t* data, size_t data_size) {
  if (!data || data_size == 0) return;
  if (data_size > seg.writer->GetBufferCapacity()) return;
  uint8_t* buf = seg.writer->GetWriteBuffer();
  if (!buf) return;
  std::memcpy(buf, data, data_size);
  SlotMetadata* meta = seg.writer->GetWriteMetadata();
  meta->timestamp_sec = sec;
  meta->timestamp_nsec = nsec;
  meta->width = width;
  meta->height = height;
  meta->data_size = data_size;
  std::strncpy(meta->encoding, encoding, 15);
  meta->encoding[15] = '\0';
  seg.writer->Publish();
}

void ShmWriter::WriteImageData(uint32_t sensor_id, int32_t sec, uint32_t nsec,
                               uint32_t height, uint32_t width,
                               const char* encoding,
                               const uint8_t* data, size_t data_size) {
  auto it = segments_.find(sensor_id);
  if (it == segments_.end()) return;
  WriteData(it->second, sec, nsec, width, height, encoding, data, data_size);
}

void ShmWriter::WriteLidarData(uint32_t sensor_id, int32_t sec, uint32_t nsec,
                               const uint8_t* data, size_t data_size,
                               uint32_t width, uint32_t height,
                               const char* encoding) {
  auto it = segments_.find(sensor_id);
  if (it == segments_.end()) return;
  WriteData(it->second, sec, nsec, width, height, encoding, data, data_size);
}

}  // namespace agnocast
}  // namespace ros2
}  // namespace carla
