// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <gtest/gtest.h>
#include "carla/ros2/data_types.h"

using namespace carla::ros2;

TEST(QoSConversion, TopicConfigDefaultValues) {
    TopicConfig config{};
    config.domain_id = 42;
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 10;

    ASSERT_EQ(config.domain_id, 42u);
    ASSERT_EQ(config.history_qos_depth, 10);
}

TEST(QoSConversion, AllReliabilityValues) {
    TopicConfig config{};
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    ASSERT_EQ(config.reliability_qos, ReliabilityQoS::RELIABLE);
    config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
    ASSERT_EQ(config.reliability_qos, ReliabilityQoS::BEST_EFFORT);
}

TEST(QoSConversion, AllDurabilityValues) {
    TopicConfig config{};
    config.durability_qos = DurabilityQoS::TRANSIENT_LOCAL;
    ASSERT_EQ(config.durability_qos, DurabilityQoS::TRANSIENT_LOCAL);
    config.durability_qos = DurabilityQoS::VOLATILE;
    ASSERT_EQ(config.durability_qos, DurabilityQoS::VOLATILE);
}

TEST(QoSConversion, AllHistoryValues) {
    TopicConfig config{};
    config.history_qos = HistoryQoS::KEEP_LAST;
    ASSERT_EQ(config.history_qos, HistoryQoS::KEEP_LAST);
    config.history_qos = HistoryQoS::KEEP_ALL;
    ASSERT_EQ(config.history_qos, HistoryQoS::KEEP_ALL);
}

TEST(QoSConversion, SuffixHandling) {
    TopicConfig config{};
    config.suffix = "/image_raw";
    ASSERT_EQ(config.suffix, "/image_raw");

    TopicConfig config2{};
    ASSERT_EQ(config2.suffix, "");
}
