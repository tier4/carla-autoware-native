// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <gtest/gtest.h>

#include <carla/ros2/ImuPacing.h>

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using carla::ros2::ImuPacer;
using carla::ros2::ImuPacingConfig;
using carla::ros2::ImuPacingOffsets;

namespace {

constexpr double kTol = 1e-9;

ImuPacingConfig Parsed(const std::string &text) {
  ImuPacingConfig config;
  EXPECT_TRUE(ImuPacingConfig::Parse(text, config)) << text;
  return config;
}

double Now() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// Records each task's run time; can signal when the first one has run.
class Recorder {
public:
  ImuPacer::Task Task(int id) {
    return [this, id] {
      std::lock_guard<std::mutex> lock(_mutex);
      _runs.emplace_back(id, Now());
      if (!_first_set) {
        _first_set = true;
        _first.set_value();
      }
    };
  }
  void WaitFirst() { _first_future.wait(); }
  std::vector<std::pair<int, double>> Runs() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _runs;
  }

private:
  std::mutex _mutex;
  std::vector<std::pair<int, double>> _runs;
  std::promise<void> _first;
  std::future<void> _first_future = _first.get_future();
  bool _first_set = false;
};

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
  for (const char *text : {"abc", "-5", "0", "1.5", "substeps", "100us", "1234567890"}) {
    ImuPacingConfig config;
    EXPECT_FALSE(ImuPacingConfig::Parse(text, config)) << text;
  }
}

TEST(ImuPacing, off_releases_the_whole_batch_at_once) {
  const auto offsets = ImuPacingOffsets(Parsed("off"), {0.015, 0.010, 0.005, 0.0}, 0.020);
  ASSERT_EQ(offsets.size(), 4u);
  for (double offset : offsets) {
    EXPECT_NEAR(offset, 0.0, kTol);
  }
}

TEST(ImuPacing, substep_follows_the_sample_times) {
  // Lags = how long before the frame stamp each sample was taken.
  const auto uniform = ImuPacingOffsets(Parsed("substep"), {0.015, 0.010, 0.005, 0.0}, 0.0);
  ASSERT_EQ(uniform.size(), 4u);
  for (size_t k = 0; k < uniform.size(); ++k) {
    EXPECT_NEAR(uniform[k], 0.005 * static_cast<double>(k), kTol);
  }
  // Uneven substeps keep their own spacing (2 / 8 / 2 / 8 ms).
  const auto uneven = ImuPacingOffsets(Parsed("substep"), {0.018, 0.010, 0.008, 0.0}, 0.0);
  ASSERT_EQ(uneven.size(), 4u);
  EXPECT_NEAR(uneven[1], 0.008, kTol);
  EXPECT_NEAR(uneven[2], 0.010, kTol);
  EXPECT_NEAR(uneven[3], 0.018, kTol);
}

TEST(ImuPacing, spacing_is_compressed_to_fit_the_previous_frame) {
  // Four 5 ms samples need 20 ms; the previous frame took 10 ms: half the spacing.
  const auto offsets = ImuPacingOffsets(Parsed("substep"), {0.015, 0.010, 0.005, 0.0}, 0.010);
  ASSERT_EQ(offsets.size(), 4u);
  EXPECT_NEAR(offsets[3], 0.0075, kTol);
  // A frame longer than the batch needs leaves the spacing as it is.
  const auto slow = ImuPacingOffsets(Parsed("substep"), {0.015, 0.010, 0.005, 0.0}, 0.050);
  EXPECT_NEAR(slow[3], 0.015, kTol);
}

TEST(ImuPacing, fixed_interval_ignores_the_sample_times) {
  const std::vector<double> lags = {0.0286, 0.0238, 0.0190, 0.0143, 0.0095, 0.0048, 0.0};
  const auto offsets = ImuPacingOffsets(Parsed("1000"), lags, 0.0333);
  ASSERT_EQ(offsets.size(), 7u);
  EXPECT_NEAR(offsets[6], 0.006, kTol);
}

TEST(ImuPacing, empty_batch_gives_no_offsets) {
  EXPECT_TRUE(ImuPacingOffsets(Parsed("substep"), {}, 0.02).empty());
}

TEST(ImuPacer, releases_in_order_and_spaced) {
  ImuPacer pacer;
  pacer.Start();
  Recorder rec;
  const double t0 = Now();
  pacer.SubmitBatch({rec.Task(0), rec.Task(1), rec.Task(2)}, {0.0, 0.020, 0.040});
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  pacer.Stop();
  const auto runs = rec.Runs();
  ASSERT_EQ(runs.size(), 3u);
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(runs[k].first, k);
  }
  EXPECT_GE(runs[1].second - t0, 0.019);
  EXPECT_GE(runs[2].second - t0, 0.039);
  // Spaced, not run together.
  EXPECT_GE(runs[1].second - runs[0].second, 0.015);
  EXPECT_GE(runs[2].second - runs[1].second, 0.015);
}

TEST(ImuPacer, a_new_batch_pulls_the_rest_forward_but_keeps_the_minimum_gap) {
  ImuPacer pacer;
  pacer.SetMinGap(0.005);
  pacer.Start();
  Recorder rec;
  const double t0 = Now();
  pacer.SubmitBatch({rec.Task(0), rec.Task(1), rec.Task(2)}, {0.0, 1.0, 2.0});
  rec.WaitFirst();
  pacer.SubmitBatch({rec.Task(3)}, {0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  pacer.Stop();
  const auto runs = rec.Runs();
  ASSERT_EQ(runs.size(), 4u);
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(runs[k].first, k);
  }
  // 1 and 2 did not wait for their 1 s / 2 s offsets ...
  EXPECT_LT(runs[3].second - t0, 0.5);
  // ... but never ran closer than the minimum gap.
  for (int k = 1; k < 4; ++k) {
    EXPECT_GE(runs[k].second - runs[k - 1].second, 0.0045) << k;
  }
}

TEST(ImuPacer, pending_work_is_bounded_dropping_the_oldest) {
  ImuPacer pacer;
  pacer.SetMaxPending(3);
  pacer.Start();
  Recorder rec;
  pacer.SubmitBatch(
      {rec.Task(0), rec.Task(1), rec.Task(2), rec.Task(3), rec.Task(4)},
      {10.0, 10.0, 10.0, 10.0, 10.0});
  EXPECT_EQ(pacer.DroppedCount(), 2u);
  EXPECT_EQ(pacer.PendingCount(), 3u);
  pacer.Stop();
  EXPECT_TRUE(rec.Runs().empty());
}

TEST(ImuPacer, clear_drops_pending_and_keeps_running) {
  ImuPacer pacer;
  pacer.Start();
  Recorder rec;
  pacer.SubmitBatch({rec.Task(0), rec.Task(1)}, {0.0, 0.300});
  rec.WaitFirst();
  pacer.Clear();
  pacer.SubmitBatch({rec.Task(2)}, {0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  pacer.Stop();
  const auto runs = rec.Runs();
  ASSERT_EQ(runs.size(), 2u);
  EXPECT_EQ(runs[0].first, 0);
  EXPECT_EQ(runs[1].first, 2);
}

TEST(ImuPacer, stop_drops_what_is_pending) {
  ImuPacer pacer;
  pacer.Start();
  Recorder rec;
  pacer.SubmitBatch({rec.Task(0), rec.Task(1)}, {0.0, 0.300});
  rec.WaitFirst();
  pacer.Stop();
  EXPECT_EQ(rec.Runs().size(), 1u);
  // Submitting after Stop does nothing.
  pacer.SubmitBatch({rec.Task(2)}, {0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(rec.Runs().size(), 1u);
}

TEST(ImuPacer, a_throwing_task_does_not_stop_the_pacer) {
  ImuPacer pacer;
  pacer.Start();
  Recorder rec;
  pacer.SubmitBatch({[] { throw std::runtime_error("boom"); }, rec.Task(1)}, {0.0, 0.0});
  rec.WaitFirst();
  pacer.Stop();
  ASSERT_EQ(rec.Runs().size(), 1u);
  EXPECT_EQ(rec.Runs()[0].first, 1);
}
