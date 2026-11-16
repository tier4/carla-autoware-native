// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// Exposes the zenoh-c-free wire-format helpers (ZenohWireFormat.h) without a
// middleware; QosProfile.h itself has no middleware dependency.
#define CARLA_ROS2_MIDDLEWARE_TESTING

#include "test.h"

#include <carla/ros2/middleware/QosProfile.h>
#include <carla/ros2/middleware/zenoh/ZenohWireFormat.h>

#include <string>

using carla::ros2::QosProfile;

namespace {

constexpr const char *kAllEightVolatile =
    "reliability=reliable,durability=volatile,history=keep_last,depth=1,"
    "deadline=inf,lifespan=inf,liveliness=automatic,lease=inf";

QosProfile Parse(const std::string &spec) {
  QosProfile p;
  EXPECT_TRUE(QosProfile::FromString(spec, p)) << spec;
  return p;
}

bool Rejects(const std::string &spec) {
  QosProfile p = QosProfile::ReliableTransientLocal();
  p.history_depth = 7;
  const QosProfile before = p;
  const bool ok = QosProfile::FromString(spec, p);
  // A rejected spec leaves the output untouched.
  EXPECT_EQ(p, before) << spec;
  return !ok;
}

} // namespace

TEST(ros2_qos_profile, defaults_are_the_dds_writer_defaults) {
  const QosProfile p;
  EXPECT_EQ(p.reliability, QosProfile::Reliability::Reliable);
  EXPECT_EQ(p.durability, QosProfile::Durability::Volatile);
  EXPECT_EQ(p.history, QosProfile::History::KeepLast);
  EXPECT_EQ(p.history_depth, 1);
  EXPECT_EQ(p.deadline_ns, QosProfile::kInfinite);
  EXPECT_EQ(p.lifespan_ns, QosProfile::kInfinite);
  EXPECT_EQ(p.liveliness, QosProfile::Liveliness::Automatic);
  EXPECT_EQ(p.liveliness_lease_ns, QosProfile::kInfinite);
  EXPECT_FALSE(p.HasTimedPolicies());
  EXPECT_EQ(p.ToString(), kAllEightVolatile);
}

TEST(ros2_qos_profile, short_form_is_unchanged) {
  QosProfile expected;
  EXPECT_EQ(Parse("reliable"), expected);
  expected.reliability = QosProfile::Reliability::BestEffort;
  expected.history_depth = 5;
  EXPECT_EQ(Parse("best_effort:5"), expected);
  EXPECT_TRUE(Rejects("fast"));
  EXPECT_TRUE(Rejects("reliable:"));
  EXPECT_TRUE(Rejects("reliable:0"));
  EXPECT_TRUE(Rejects("reliable:100001"));
  EXPECT_TRUE(Rejects("reliable:2x"));
  EXPECT_TRUE(Rejects(""));
}

TEST(ros2_qos_profile, all_eight_policies_round_trip) {
  const QosProfile p = Parse(kAllEightVolatile);
  EXPECT_EQ(p, QosProfile::ReliableVolatile());
  EXPECT_EQ(p.ToString(), kAllEightVolatile);

  const std::string timed =
      "reliability=best_effort,durability=transient_local,history=keep_all,depth=10,"
      "deadline=0.1,lifespan=2,liveliness=manual_by_topic,lease=1.5";
  const QosProfile q = Parse(timed);
  EXPECT_EQ(q.reliability, QosProfile::Reliability::BestEffort);
  EXPECT_EQ(q.durability, QosProfile::Durability::TransientLocal);
  EXPECT_EQ(q.history, QosProfile::History::KeepAll);
  EXPECT_EQ(q.history_depth, 10);
  EXPECT_EQ(q.deadline_ns, 100000000);
  EXPECT_EQ(q.lifespan_ns, 2000000000);
  EXPECT_EQ(q.liveliness, QosProfile::Liveliness::ManualByTopic);
  EXPECT_EQ(q.liveliness_lease_ns, 1500000000);
  EXPECT_TRUE(q.HasTimedPolicies());
  EXPECT_EQ(q.ToString(), timed);
  EXPECT_EQ(Parse(q.ToString()), q);
}

TEST(ros2_qos_profile, keys_left_out_keep_their_defaults) {
  EXPECT_EQ(Parse("durability=transient_local"), QosProfile::ReliableTransientLocal());
  QosProfile expected;
  expected.deadline_ns = 250000000;
  EXPECT_EQ(Parse("deadline=0.25"), expected);
}

TEST(ros2_qos_profile, spaces_around_keys_and_values_are_ignored) {
  QosProfile expected;
  expected.reliability = QosProfile::Reliability::BestEffort;
  expected.history_depth = 3;
  EXPECT_EQ(Parse(" reliability = best_effort , depth = 3 "), expected);
}

TEST(ros2_qos_profile, durations) {
  EXPECT_EQ(Parse("lease=0.000000001").liveliness_lease_ns, 1);
  EXPECT_EQ(Parse("lifespan=2147483647.999999999").lifespan_ns, 2147483647999999999LL);
  EXPECT_EQ(Parse("deadline=inf").deadline_ns, QosProfile::kInfinite);
  EXPECT_NE(Parse("deadline=1.50").ToString().find(",deadline=1.5,"), std::string::npos);
  EXPECT_TRUE(Rejects("deadline=0"));
  EXPECT_TRUE(Rejects("deadline=0.0"));
  EXPECT_TRUE(Rejects("deadline=0.0000000001"));
  EXPECT_TRUE(Rejects("deadline=-1"));
  EXPECT_TRUE(Rejects("deadline=1e3"));
  EXPECT_TRUE(Rejects("deadline=2147483648"));
  EXPECT_TRUE(Rejects("deadline=.5"));
  EXPECT_TRUE(Rejects("deadline=5."));
  EXPECT_TRUE(Rejects("deadline=INF"));
  EXPECT_TRUE(Rejects("deadline=infinite"));
}

TEST(ros2_qos_profile, malformed_key_value_specs_are_rejected) {
  EXPECT_TRUE(Rejects("reliability=reliable,"));
  EXPECT_TRUE(Rejects("reliability=reliable,,depth=1"));
  EXPECT_TRUE(Rejects("reliability=reliable,depth"));
  EXPECT_TRUE(Rejects("reliability="));
  EXPECT_TRUE(Rejects("=reliable"));
  EXPECT_TRUE(Rejects("reliability=reliable,reliability=best_effort"));
  EXPECT_TRUE(Rejects("reliability=Reliable"));
  EXPECT_TRUE(Rejects("durability=transient"));
  EXPECT_TRUE(Rejects("durability=persistent"));
  EXPECT_TRUE(Rejects("history=keep"));
  EXPECT_TRUE(Rejects("depth=0"));
  EXPECT_TRUE(Rejects("depth=+5"));
  EXPECT_TRUE(Rejects("liveliness=manual_by_node"));
  EXPECT_TRUE(Rejects("liveliness_lease_duration=inf"));
  EXPECT_TRUE(Rejects("partition=a"));
}

TEST(ros2_qos_profile, zenoh_keyexpr_keeps_the_default_segment) {
  EXPECT_EQ(carla::ros2::zenoh_qos_keyexpr(QosProfile{}), "1:2:1,1:,:,:,,");
  EXPECT_EQ(carla::ros2::zenoh_qos_keyexpr(QosProfile::ReliableTransientLocal()), "1:1:1,1:,:,:,,");
  EXPECT_EQ(carla::ros2::zenoh_qos_keyexpr(Parse(kAllEightVolatile)), "1:2:1,1:,:,:,,");
}

TEST(ros2_qos_profile, zenoh_keyexpr_encodes_the_timed_policies) {
  const QosProfile q = Parse(
      "reliability=best_effort,durability=volatile,history=keep_last,depth=4,"
      "deadline=0.1,lifespan=2,liveliness=manual_by_topic,lease=1.5");
  EXPECT_EQ(carla::ros2::zenoh_qos_keyexpr(q), "2:2:1,4:0,100000000:2,0:3,1,500000000");
}
