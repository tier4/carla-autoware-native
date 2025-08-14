#include "AutowarePublisher.h"

#include "AutowarePublisherBase.hpp"

#include "carla/ros2/types/VelocityReport.h"
#include "carla/ros2/types/VelocityReportPubSubTypes.h"

namespace carla {
namespace ros2 {

// Autoware specific publisher classes
class VelocityReportPublisher
: public AutowarePublisherBase<autoware_vehicle_msgs::msg::VelocityReport, autoware_vehicle_msgs::msg::VelocityReportPubSubType>
{
public:
  VelocityReportPublisher() : AutowarePublisherBase("base_line", "", "/vehicle/status/velocity_status") {}
  const char * type() const override { return "Autoware velocity report"; }
};


class AutowarePublisher::Implementation
{
public:
  VelocityReportPublisher _velocity_publisher{};
};

AutowarePublisher::AutowarePublisher(void* vehicle, const CarlaPublisher::DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  _impl->_velocity_publisher.Init(domain_id);
}

void AutowarePublisher::SetVelocity(const int32_t seconds, const uint32_t nanoseconds,
  const float longitudinal_velocity, const float lateral_velocity, const float heading_rate)
{
  builtin_interfaces::msg::Time time;
  time.sec(seconds);
  time.nanosec(nanoseconds);

  std_msgs::msg::Header header;
  header.stamp(std::move(time));
  header.frame_id(_impl->_velocity_publisher.frame_id());

  autoware_vehicle_msgs::msg::VelocityReport report;
  report.header(std::move(header));
  report.longitudinal_velocity(longitudinal_velocity);
  report.lateral_velocity(lateral_velocity);
  report.heading_rate(heading_rate);

  _impl->_velocity_publisher.SetData(report);
}

void AutowarePublisher::Publish()
{
  _impl->_velocity_publisher.Publish();
}

}  // namespace carla
}  // namespace ros2
