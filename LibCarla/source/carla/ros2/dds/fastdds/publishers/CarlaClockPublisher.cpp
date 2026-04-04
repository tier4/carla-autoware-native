#include "CarlaClockPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "Clock.h"

namespace carla {
namespace ros2 {

  struct CarlaClockPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    rosgraph::msg::Clock _clock {};
  };

  bool CarlaClockPublisher::Init(const TopicConfig& config) {
    _impl->_dds = CreateDDSPublisher("rosgraph_msgs::msg::Clock");
    if (!_impl->_dds) return false;

    const std::string topic_name { "rt/clock" };

    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaClockPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_clock);
  }

  void CarlaClockPublisher::SetData(int32_t sec, uint32_t nanosec) {
    _impl->_clock.clock().sec(sec);
    _impl->_clock.clock().nanosec(nanosec);
  }

  CarlaClockPublisher::CarlaClockPublisher(const char* ros_name, const char* parent) :
  _impl(std::make_shared<CarlaClockPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
  }

  CarlaClockPublisher::~CarlaClockPublisher() = default;
  CarlaClockPublisher::CarlaClockPublisher(const CarlaClockPublisher&) = default;
  CarlaClockPublisher& CarlaClockPublisher::operator=(const CarlaClockPublisher&) = default;
  CarlaClockPublisher::CarlaClockPublisher(CarlaClockPublisher&&) = default;
  CarlaClockPublisher& CarlaClockPublisher::operator=(CarlaClockPublisher&&) = default;
}
}
