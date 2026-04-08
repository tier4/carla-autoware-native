// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>
#include <optional>
#include "carla/ros2/data_types.h"

namespace carla {
namespace ros2 {

  class CarlaPublisher {
    public:
      const std::string& frame_id() const { return _frame_id; }
      const std::string& topic_name() const { return _topic_name; }
      const std::string& name() const { return _name; }
      const std::string& parent() const { return _parent; }

      void frame_id(std::string&& frame_id) { _frame_id = std::move(frame_id); }
      void topic_name(std::string&& topic_name) { _topic_name = std::move(topic_name); }
      void name(std::string&& name) { _name = std::move(name); }
      void parent(std::string&& parent) { _parent = std::move(parent); }

      /// @return user specified valid FastDDS topic name
      std::optional<std::string> ValidTopicName(const std::string& suffix = "") const {
        if (_topic_name.empty()) {
          return std::nullopt;
        }
        std::string topic_name = "rt";
        if (_topic_name.front() != '/') {
          topic_name += "/";
        }
        topic_name += _topic_name;

        if (!suffix.empty() && suffix.front() != '/') {
          topic_name += "/";
        }
        topic_name += suffix;
        return topic_name;
      }

      virtual const char* type() const = 0;

    public:
      CarlaPublisher() = default;
      virtual ~CarlaPublisher() = default;

    protected:
      std::string _frame_id = "";
      std::string _topic_name = "";
      std::string _name = "";
      std::string _parent = "";
  };
}
}
