#pragma once

#include "ShmProtocol.h"
#include <cstring>
#include <atomic>

namespace carla {
namespace ros2 {
namespace agnocast {

class TripleBufferWriter {
public:
  explicit TripleBufferWriter(void* shm_base)
    : base_(shm_base)
    , header_(static_cast<ShmHeader*>(shm_base))
    , write_slot_(NextFreeSlot(header_->latest_slot.load(std::memory_order_relaxed))) {}

  uint8_t* GetWriteBuffer() {
    return GetSlotData(base_, write_slot_, header_->buffer_capacity);
  }

  SlotMetadata* GetWriteMetadata() {
    return GetSlotMetadata(base_, write_slot_, header_->buffer_capacity);
  }

  uint64_t GetBufferCapacity() const {
    return header_->buffer_capacity;
  }

  void Publish() {
    uint32_t old_latest = header_->latest_slot.exchange(write_slot_, std::memory_order_release);
    header_->write_frame_id.fetch_add(1, std::memory_order_relaxed);
    write_slot_ = old_latest;
  }

private:
  static uint32_t NextFreeSlot(uint32_t latest) {
    return (latest + 1) % NUM_SLOTS;
  }

  void* base_;
  ShmHeader* header_;
  uint32_t write_slot_;
};

class TripleBufferReader {
public:
  explicit TripleBufferReader(const void* shm_base)
    : base_(shm_base)
    , header_(static_cast<const ShmHeader*>(shm_base))
    , last_read_frame_(0) {}

  bool HasNewFrame() const {
    return header_->write_frame_id.load(std::memory_order_acquire) > last_read_frame_;
  }

  uint32_t GetLatestSlot() const {
    return header_->latest_slot.load(std::memory_order_acquire);
  }

  const SlotMetadata* ReadMetadata() {
    uint32_t slot = GetLatestSlot();
    return GetSlotMetadata(base_, slot, header_->buffer_capacity);
  }

  const uint8_t* ReadData() {
    uint32_t slot = GetLatestSlot();
    return GetSlotData(base_, slot, header_->buffer_capacity);
  }

  void MarkRead() {
    last_read_frame_ = header_->write_frame_id.load(std::memory_order_acquire);
  }

  uint64_t GetBufferCapacity() const {
    return header_->buffer_capacity;
  }

  const ShmHeader* GetHeader() const {
    return header_;
  }

private:
  const void* base_;
  const ShmHeader* header_;
  uint64_t last_read_frame_;
};

}  // namespace agnocast
}  // namespace ros2
}  // namespace carla
