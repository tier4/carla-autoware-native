// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <gtest/gtest.h>

#include <carla/ros2/ImuPacing.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using carla::ros2::ImuPacer;
using carla::ros2::ImuPacingConfig;
using carla::ros2::ImuPacingOffsets;

namespace {

constexpr double kTol = 1e-12;

ImuPacingConfig Parsed(const std::string &text) {
  ImuPacingConfig config;
  EXPECT_TRUE(ImuPacingConfig::Parse(text, config)) << text;
  return config;
}

double Now() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

TEST(ImuPacing, parse_accepts_off_substep_and_microseconds) {
  EXPECT_EQ(Parsed("").mode, ImuPacingConfig::Mode::Off);
  EXPECT_EQ(Parsed("off").mode, ImuPacingConfig::Mode::Off);
  EXPECT_EQ(Parsed("substep").mode, ImuPacingConfig::Mode::Substep);
  const ImuPacingConfig fixed = Parsed("1000");
  EXPECT_EQ(fixed.mode, ImuPacingConfig::Mode::Fixed);
  EXPECT_NEAR(fixed.interval_s, 0.001, kTol);
}

TEST(ImuPacing, parse_rejects_other_values) {
  for (const char *text : {"abc", "-5", "0", "1.5", "substeps", "100us"}) {
    ImuPacingConfig config;
    EXPECT_FALSE(ImuPacingConfig::Parse(text, config)) << text;
  }
}

TEST(ImuPacing, off_releases_the_whole_batch_at_once) {
  const auto offsets = ImuPacingOffsets(Parsed("off"), 4, 0.005, 0.020);
  ASSERT_EQ(offsets.size(), 4u);
  for (double offset : offsets) {
    EXPECT_NEAR(offset, 0.0, kTol);
  }
}

TEST(ImuPacing, substep_spaces_samples_by_the_substep) {
  const auto offsets = ImuPacingOffsets(Parsed("substep"), 4, 0.005, 0.0);
  ASSERT_EQ(offsets.size(), 4u);
  for (size_t k = 0; k < offsets.size(); ++k) {
    EXPECT_NEAR(offsets[k], 0.005 * static_cast<double>(k), kTol);
  }
}

TEST(ImuPacing, spacing_is_compressed_to_fit_the_previous_frame) {
  // The previous frame took 10 ms of wall time: 4 samples get 2.5 ms each.
  const auto offsets = ImuPacingOffsets(Parsed("substep"), 4, 0.005, 0.010);
  ASSERT_EQ(offsets.size(), 4u);
  EXPECT_NEAR(offsets[3], 0.0075, kTol);
  // A frame longer than the batch needs leaves the spacing as configured.
  const auto slow = ImuPacingOffsets(Parsed("substep"), 4, 0.005, 0.050);
  EXPECT_NEAR(slow[3], 0.015, kTol);
}

TEST(ImuPacing, fixed_interval_ignores_the_substep) {
  const auto offsets = ImuPacingOffsets(Parsed("1000"), 7, 0.004761, 0.0333);
  ASSERT_EQ(offsets.size(), 7u);
  EXPECT_NEAR(offsets[6], 0.006, kTol);
}

TEST(ImuPacing, empty_batch_gives_no_offsets) {
  EXPECT_TRUE(ImuPacingOffsets(Parsed("substep"), 0, 0.005, 0.02).empty());
}

TEST(ImuPacer, releases_in_order_at_the_offsets) {
  ImuPacer pacer;
  pacer.Start();
  std::mutex mutex;
  std::vector<std::pair<int, double>> seen;
  const double t0 = Now();
  std::vector<ImuPacer::Task> tasks;
  for (int k = 0; k < 3; ++k) {
    tasks.push_back([&, k] {
      std::lock_guard<std::mutex> lock(mutex);
      seen.emplace_back(k, Now() - t0);
    });
  }
  pacer.SubmitBatch(std::move(tasks), {0.0, 0.020, 0.040});
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  pacer.Stop();
  ASSERT_EQ(seen.size(), 3u);
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(seen[k].first, k);
  }
  EXPECT_GE(seen[1].second, 0.019);
  EXPECT_GE(seen[2].second, 0.039);
}

TEST(ImuPacer, a_new_batch_first_releases_the_rest_of_the_previous_one) {
  ImuPacer pacer;
  pacer.Start();
  std::mutex mutex;
  std::vector<std::pair<std::string, double>> seen;
  const double t0 = Now();
  auto task = [&](std::string name) {
    return [&, name] {
      std::lock_guard<std::mutex> lock(mutex);
      seen.emplace_back(name, Now() - t0);
    };
  };
  pacer.SubmitBatch({task("a0"), task("a1"), task("a2")}, {0.0, 0.200, 0.400});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  pacer.SubmitBatch({task("b0")}, {0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  pacer.Stop();
  ASSERT_EQ(seen.size(), 4u);
  EXPECT_EQ(seen[0].first, "a0");
  EXPECT_EQ(seen[1].first, "a1");
  EXPECT_EQ(seen[2].first, "a2");
  EXPECT_EQ(seen[3].first, "b0");
  // a1 and a2 did not wait for their 200 / 400 ms offsets.
  EXPECT_LT(seen[2].second, 0.150);
}

TEST(ImuPacer, stop_drops_what_is_pending) {
  ImuPacer pacer;
  pacer.Start();
  std::atomic<int> count{0};
  pacer.SubmitBatch({[&] { ++count; }, [&] { ++count; }}, {0.0, 0.300});
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  pacer.Stop();
  EXPECT_EQ(count.load(), 1);
  // Submitting after Stop does nothing.
  pacer.SubmitBatch({[&] { ++count; }}, {0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(count.load(), 1);
}
