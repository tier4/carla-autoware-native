// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

// Paced ROS 2 publishing for sensors that produce several samples per frame
// (sensor.other.imu_highprecision: one per physics substep). The samples of a
// frame are computed together, so by default they are published back to back.
// A subscriber with a history depth of 1 (as a driver of a real IMU, which
// publishes evenly, would allow) then keeps only the last one. Pacing spreads
// the publishes of a frame over wall time; the header stamps are not changed.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace carla {
namespace ros2 {

  /// The value of -ros2-imu-pacing: "off" (or empty; back to back, the
  /// default), "substep" (spaced by the physics substep) or a positive whole
  /// number of microseconds (a fixed spacing).
  struct ImuPacingConfig {
    enum class Mode { Off, Substep, Fixed };
    Mode mode = Mode::Off;
    double interval_s = 0.0;  ///< Fixed only.

    static bool Parse(const std::string &text, ImuPacingConfig &out) {
      ImuPacingConfig config;
      if (text.empty() || text == "off") {
        out = config;
        return true;
      }
      if (text == "substep") {
        config.mode = Mode::Substep;
        out = config;
        return true;
      }
      if (text.size() > 9u ||
          !std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; })) {
        return false;
      }
      const long microseconds = std::stol(text);
      if (microseconds <= 0) {
        return false;
      }
      config.mode = Mode::Fixed;
      config.interval_s = 1e-6 * static_cast<double>(microseconds);
      out = config;
      return true;
    }
  };

  /// Release offsets in seconds, from the moment a frame's batch of `count`
  /// samples is handed over. `substep_s` is the batch's sample spacing in
  /// simulation time; `frame_span_s` is the wall time the previous frame took
  /// (<= 0 if unknown). The spacing is compressed so that the whole batch is
  /// released within that span, ahead of the next frame's batch.
  inline std::vector<double> ImuPacingOffsets(
      const ImuPacingConfig &config,
      std::size_t count,
      double substep_s,
      double frame_span_s) {
    std::vector<double> offsets(count, 0.0);
    if (count == 0u || config.mode == ImuPacingConfig::Mode::Off) {
      return offsets;
    }
    double interval = config.mode == ImuPacingConfig::Mode::Substep ? substep_s : config.interval_s;
    if (frame_span_s > 0.0) {
      interval = std::min(interval, frame_span_s / static_cast<double>(count));
    }
    interval = std::max(interval, 0.0);
    for (std::size_t k = 0u; k < count; ++k) {
      offsets[k] = interval * static_cast<double>(k);
    }
    return offsets;
  }

  /// Runs submitted tasks on its own thread, each at its release time. A new
  /// batch first releases whatever is left of the previous one (in order), so
  /// nothing piles up when frames come faster than their batches drain. Stop()
  /// drops pending tasks and joins; SubmitBatch() after Stop() does nothing.
  class ImuPacer {
  public:
    using Task = std::function<void()>;

    ImuPacer() = default;
    ImuPacer(const ImuPacer &) = delete;
    ImuPacer &operator=(const ImuPacer &) = delete;
    ~ImuPacer() { Stop(); }

    void Start() {
      std::lock_guard<std::mutex> lock(_mutex);
      if (_thread.joinable()) {
        return;
      }
      _stopping = false;
      _running = true;
      _thread = std::thread([this] { Run(); });
    }

    void Stop() {
      {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_thread.joinable()) {
          _running = false;
          _queue.clear();
          return;
        }
        _stopping = true;
        _running = false;
        _queue.clear();
      }
      _cv.notify_all();
      _thread.join();
    }

    /// `offsets` (seconds from now) must have one entry per task.
    void SubmitBatch(std::vector<Task> tasks, const std::vector<double> &offsets) {
      const Clock::time_point now = Clock::now();
      {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_running) {
          return;
        }
        for (Item &item : _queue) {
          item.release = std::min(item.release, now);
        }
        for (std::size_t k = 0u; k < tasks.size(); ++k) {
          const double offset = k < offsets.size() ? offsets[k] : 0.0;
          _queue.push_back(Item{
              now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(offset)),
              std::move(tasks[k])});
        }
      }
      _cv.notify_all();
    }

  private:
    using Clock = std::chrono::steady_clock;

    struct Item {
      Clock::time_point release;
      Task task;
    };

    void Run() {
      std::unique_lock<std::mutex> lock(_mutex);
      while (!_stopping) {
        if (_queue.empty()) {
          _cv.wait(lock, [this] { return _stopping || !_queue.empty(); });
          continue;
        }
        const Clock::time_point release = _queue.front().release;
        if (Clock::now() < release) {
          _cv.wait_until(lock, release);
          continue;  // re-check: a new batch may have pulled the release forward
        }
        Task task = std::move(_queue.front().task);
        _queue.pop_front();
        lock.unlock();
        if (task) {
          task();
        }
        lock.lock();
      }
    }

    std::mutex _mutex;
    std::condition_variable _cv;
    std::deque<Item> _queue;
    std::thread _thread;
    bool _stopping = false;
    bool _running = false;
  };

} // namespace ros2
} // namespace carla
