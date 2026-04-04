#include "CarlaLineInvasionPublisher.h"

#include <string>

#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "CarlaLineInvasion.h"

namespace carla {
namespace ros2 {

  struct CarlaLineInvasionPublisherImpl {
    std::unique_ptr<DDSPublisherImpl> _dds;
    carla_msgs::msg::LaneInvasionEvent _event {};
  };

  bool CarlaLineInvasionPublisher::Init(const DomainId domain_id) {
    _impl->_dds = CreateDDSPublisher("carla_msgs::msg::LaneInvasionEvent");
    if (!_impl->_dds) return false;

    const std::string base { "rt/carla/" };
    std::string topic_name = base;
    if (!_parent.empty())
      topic_name += _parent + "/";
    topic_name += _name;
    if (const auto custom_topic_name = ValidTopicName()) {
      topic_name = custom_topic_name.value();
    }

    TopicConfig config;
    config.domain_id = domain_id;
    if (!_impl->_dds->Init(config, _name, topic_name, /*use_preallocated_realloc=*/false)) {
      return false;
    }
    _frame_id = _name;
    return true;
  }

  bool CarlaLineInvasionPublisher::Publish() {
    return _impl->_dds->Write(&_impl->_event);
  }

  void CarlaLineInvasionPublisher::SetData(int32_t seconds, uint32_t nanoseconds, const int32_t* data) {
    builtin_interfaces::msg::Time time;
    time.sec(seconds);
    time.nanosec(nanoseconds);

    std_msgs::msg::Header header;
    header.stamp(std::move(time));
    header.frame_id(_frame_id);

    _impl->_event.header(std::move(header));
    _impl->_event.crossed_lane_markings({data[0], data[1], data[2]});
  }

  CarlaLineInvasionPublisher::CarlaLineInvasionPublisher(const char* ros_name, const char* parent, const char* ros_topic_name) :
  _impl(std::make_shared<CarlaLineInvasionPublisherImpl>()) {
    _name = ros_name;
    _parent = parent;
    _topic_name = ros_topic_name;
  }

  CarlaLineInvasionPublisher::~CarlaLineInvasionPublisher() = default;
  CarlaLineInvasionPublisher::CarlaLineInvasionPublisher(const CarlaLineInvasionPublisher&) = default;
  CarlaLineInvasionPublisher& CarlaLineInvasionPublisher::operator=(const CarlaLineInvasionPublisher&) = default;
  CarlaLineInvasionPublisher::CarlaLineInvasionPublisher(CarlaLineInvasionPublisher&&) = default;
  CarlaLineInvasionPublisher& CarlaLineInvasionPublisher::operator=(CarlaLineInvasionPublisher&&) = default;
}
}
