// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/middleware/QosProfile.h"
#include "carla/ros2/publishers/BasePublisher.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace carla {
namespace ros2 {

// Forward declarations keep the FastDDS-heavy PublisherImpl<> definition out of
// the main carla-server compile unit. The full template definition + Traits
// instantiations live in CarlaCameraPublisher.cpp, which is built by the
// carla-ros2-native ExternalProject (where FastDDS headers are on the include
// path). Same pattern as AckermannControlSubscriber.{h,cpp} from PR-2.
template <typename Traits> class PublisherImpl;
struct CarlaCameraImageMsgTraits;
struct CarlaCameraInfoMsgTraits;

class CarlaCameraPublisher : public BasePublisher {
public:
  /// image_topic_suffix (tier4): leaf of the image topic, "image" as upstream
  /// unless ROS2::SetCameraImageTopicSuffix changed it (e.g. "image_raw").
  CarlaCameraPublisher(
      std::string base_topic_name,
      std::string frame_id,
      const std::string &image_topic_suffix = "image",
      const std::optional<QosProfile> &qos = std::nullopt);
  ~CarlaCameraPublisher() override;

  CarlaCameraPublisher(const CarlaCameraPublisher &) = delete;
  CarlaCameraPublisher &operator=(const CarlaCameraPublisher &) = delete;
  CarlaCameraPublisher(CarlaCameraPublisher &&) noexcept = default;
  CarlaCameraPublisher &operator=(CarlaCameraPublisher &&) noexcept = default;

  bool Publish() override;

  bool WriteCameraInfo(
      int32_t seconds,
      uint32_t nanoseconds,
      uint32_t x_offset,
      uint32_t y_offset,
      uint32_t height,
      uint32_t width,
      float fov,
      bool do_rectify);

  bool WriteImage(
      int32_t seconds,
      uint32_t nanoseconds,
      uint32_t height,
      uint32_t width,
      const uint8_t *data);

protected:
  [[nodiscard]] virtual uint8_t GetChannels() const = 0;
  [[nodiscard]] virtual std::string GetEncoding() const = 0;
  [[nodiscard]] virtual std::vector<uint8_t> ComputeImage(
      uint32_t height, uint32_t width, const uint8_t *data) const;

private:
  bool WriteImage(
      int32_t seconds,
      uint32_t nanoseconds,
      uint32_t height,
      uint32_t width,
      std::vector<uint8_t> data);

  std::shared_ptr<PublisherImpl<CarlaCameraImageMsgTraits>> _impl_image;
  std::shared_ptr<PublisherImpl<CarlaCameraInfoMsgTraits>> _impl_camera_info;
};

}  // namespace ros2
}  // namespace carla
