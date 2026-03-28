// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <functional>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>
#include <atomic>

namespace carla {
namespace ros2 {

/// A single-worker publish queue that drains tasks off the game thread.
/// Tasks are std::function<void()> lambdas that capture copied sensor data
/// and call the actual DDS Publish() methods.
class ROS2PublishQueue {
public:

  ROS2PublishQueue() : _running(false) {}

  ~ROS2PublishQueue() {
    Shutdown();
  }

  /// Start the worker thread. Safe to call multiple times (no-op if running).
  void Start() {
    bool expected = false;
    if (!_running.compare_exchange_strong(expected, true))
      return;
    _worker = std::thread([this]() { WorkerLoop(); });
  }

  /// Stop the worker thread and drain remaining tasks.
  void Shutdown() {
    bool expected = true;
    if (!_running.compare_exchange_strong(expected, false))
      return;
    _cv.notify_one();
    if (_worker.joinable())
      _worker.join();
  }

  /// Enqueue a publish task. Caller must ensure all captured data is
  /// owned by the lambda (copied/moved, not referenced).
  void Enqueue(std::function<void()> task) {
    {
      std::lock_guard<std::mutex> lock(_mutex);
      _queue.push(std::move(task));
    }
    _cv.notify_one();
  }

  /// Enqueue a task, discarding any pending (not yet started) tasks.
  /// Use for sensors where only the latest data matters (e.g., camera).
  void EnqueueLatest(std::function<void()> task) {
    {
      std::lock_guard<std::mutex> lock(_mutex);
      while (!_queue.empty())
        _queue.pop();
      _queue.push(std::move(task));
    }
    _cv.notify_one();
  }

  /// Returns true if the worker thread is running.
  bool IsRunning() const { return _running.load(); }

private:

  void WorkerLoop() {
    while (_running.load()) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this]() {
          return !_queue.empty() || !_running.load();
        });
        if (!_running.load() && _queue.empty())
          break;
        if (_queue.empty())
          continue;
        task = std::move(_queue.front());
        _queue.pop();
      }
      task();
    }
    // Drain remaining tasks
    std::lock_guard<std::mutex> lock(_mutex);
    while (!_queue.empty()) {
      _queue.front()();
      _queue.pop();
    }
  }

  std::atomic<bool> _running;
  std::thread _worker;
  std::mutex _mutex;
  std::condition_variable _cv;
  std::queue<std::function<void()>> _queue;
};

} // namespace ros2
} // namespace carla
