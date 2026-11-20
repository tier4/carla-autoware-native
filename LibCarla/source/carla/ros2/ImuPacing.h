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

  /// Release offsets in seconds, from the moment a frame's batch is handed
  /// over, for samples taken `sample_lag_s[k]` seconds before the frame stamp
  /// (oldest first). "substep" keeps the samples' own spacing in simulation
  /// time; a fixed interval spaces them evenly. `frame_span_s` is the wall time
  /// the previous frame took (<= 0 if unknown): the spacing is compressed so
  /// that the batch, plus one more step, fits in it, ahead of the next batch.
  inline std::vector<double> ImuPacingOffsets(
      const ImuPacingConfig &config,
      const std::vector<double> &sample_lag_s,
      double frame_span_s) {
    const std::size_t count = sample_lag_s.size();
    std::vector<double> offsets(count, 0.0);
    if (count == 0u || config.mode == ImuPacingConfig::Mode::Off) {
      return offsets;
    }
    double span = 0.0;
    if (config.mode == ImuPacingConfig::Mode::Substep) {
      for (std::size_t k = 1u; k < count; ++k) {
        offsets[k] = std::max(offsets[k - 1], sample_lag_s[0] - sample_lag_s[k]);
      }
      span = count > 1u
          ? offsets[count - 1] * static_cast<double>(count) / static_cast<double>(count - 1)
          : 0.0;
    } else {
      for (std::size_t k = 0u; k < count; ++k) {
        offsets[k] = config.interval_s * static_cast<double>(k);
      }
      span = config.interval_s * static_cast<double>(count);
    }
    if (frame_span_s > 0.0 && span > frame_span_s) {
      const double scale = frame_span_s / span;
      for (double &offset : offsets) {
        offset *= scale;
      }
    }
    return offsets;
  }

  /// Runs submitted tasks on its own thread, each at its release time and never
  /// closer than the minimum gap to the previous one. A new batch pulls the
  /// release of whatever is left of the previous one forward (in order). At
  /// most SetMaxPending() tasks wait; beyond that the oldest are dropped and
  /// counted. Clear() drops what waits; Stop() also joins (a task already
  /// running finishes first). SubmitBatch() after Stop() does nothing. Start,
  /// Stop and the setters are meant to be called from one thread.
  class ImuPacer {
  public:
    using Task = std::function<void()>;

    ImuPacer() = default;
    ImuPacer(const ImuPacer &) = delete;
    ImuPacer &operator=(const ImuPacer &) = delete;
    ~ImuPacer() { Stop(); }

    /// Minimum wall time between two task runs (seconds, default 0).
    void SetMinGap(double seconds) {
      std::lock_guard<std::mutex> lock(_mutex);
      _min_gap = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(std::max(seconds, 0.0)));
    }

    /// Most tasks that may wait (default 64); the oldest beyond it are dropped.
    void SetMaxPending(std::size_t count) {
      std::lock_guard<std::mutex> lock(_mutex);
      _max_pending = std::max<std::size_t>(count, 1u);
    }

    std::size_t DroppedCount() const {
      std::lock_guard<std::mutex> lock(_mutex);
      return _dropped;
    }

    std::size_t PendingCount() const {
      std::lock_guard<std::mutex> lock(_mutex);
      return _queue.size();
    }

    /// Drops the tasks that wait; the thread keeps running.
    void Clear() {
      std::lock_guard<std::mutex> lock(_mutex);
      _queue.clear();
    }

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
        while (_queue.size() > _max_pending) {
          _queue.pop_front();
          ++_dropped;
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
        const Clock::time_point release = std::max(_queue.front().release, _last_run + _min_gap);
        if (Clock::now() < release) {
          _cv.wait_until(lock, release);
          continue;  // re-check: a new batch may have pulled the release forward
        }
        Task task = std::move(_queue.front().task);
        _queue.pop_front();
        _last_run = Clock::now();
        lock.unlock();
        if (task) {
          try {
            task();
          } catch (...) {
            // A failed publish must not take the thread (and the process) down.
          }
        }
        lock.lock();
      }
    }

    mutable std::mutex _mutex;
    std::condition_variable _cv;
    Clock::duration _min_gap = Clock::duration::zero();
    Clock::time_point _last_run{};
    std::size_t _max_pending = 64u;
    std::size_t _dropped = 0u;
    std::deque<Item> _queue;
    std::thread _thread;
    bool _stopping = false;
    bool _running = false;
  };

} // namespace ros2
} // namespace carla
