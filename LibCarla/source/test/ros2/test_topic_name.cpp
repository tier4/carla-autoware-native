// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <gtest/gtest.h>
#include "carla/ros2/publishers/CarlaPublisher.h"

using namespace carla::ros2;

// Concrete subclass for testing (CarlaPublisher is abstract)
class TestPublisher : public CarlaPublisher {
public:
    const char* type() const override { return "test"; }
};

TEST(TopicName, EmptyTopicNameReturnsNullopt) {
    TestPublisher pub;
    // Default topic_name is empty
    auto result = pub.ValidTopicName();
    ASSERT_FALSE(result.has_value());
}

TEST(TopicName, SimpleTopicName) {
    TestPublisher pub;
    pub.topic_name(std::string("/carla/ego/lidar"));
    auto result = pub.ValidTopicName();
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value(), "rt/carla/ego/lidar");
}

TEST(TopicName, TopicNameWithoutLeadingSlash) {
    TestPublisher pub;
    pub.topic_name(std::string("carla/ego/lidar"));
    auto result = pub.ValidTopicName();
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value(), "rt/carla/ego/lidar");
}

TEST(TopicName, TopicNameWithSuffix) {
    TestPublisher pub;
    pub.topic_name(std::string("/carla/ego"));
    auto result = pub.ValidTopicName("/image");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value(), "rt/carla/ego/image");
}

TEST(TopicName, TopicNameWithSuffixNoSlash) {
    TestPublisher pub;
    pub.topic_name(std::string("/carla/ego"));
    auto result = pub.ValidTopicName("image");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value(), "rt/carla/ego/image");
}

TEST(TopicName, EmptySuffix) {
    TestPublisher pub;
    pub.topic_name(std::string("/carla/ego"));
    auto result = pub.ValidTopicName("");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value(), "rt/carla/ego");
}
