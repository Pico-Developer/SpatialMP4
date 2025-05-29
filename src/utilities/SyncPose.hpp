#pragma once
#include <deque>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

/**
 * Example:
 *
 *    SynchronizedQueue<six_dof_t> sync_queue;
 *
 *    sync_queue.addPose(timestamp, pose);
 *    sync_queue.addPose(timestamp, pose);
 *    sync_queue.addPose(timestamp, pose);
 *
 *    sync_queue.findNearestPose(timestamp, pose, time_diff);
 *
 *    sync_queue.clear();
 */

namespace Utilities {

template <typename T>
class TimestampedData {
 public:
  double timestamp;
  T data;
  TimestampedData(double ts, const T& d) : timestamp(ts), data(d) {}
};

template <typename PoseType>
class SynchronizedQueue {
 private:
  std::deque<TimestampedData<PoseType>> pose_queue;
  size_t max_pose_queue_size = 100000000;
  mutable std::mutex mtx;

 public:
  void addPose(double timestamp, const PoseType& pose) {
    std::unique_lock<std::mutex> lock(mtx);
    pose_queue.emplace_back(timestamp, pose);
    while (pose_queue.size() > max_pose_queue_size) {
      pose_queue.pop_front();
    }
  }

  std::vector<PoseType> getAllPose() const {
    std::unique_lock<std::mutex> lock(mtx);
    std::vector<PoseType> poses;
    for (auto& pose : pose_queue) {
      poses.push_back(pose.data);
    }
    return poses;
  }

  void printAll() const {
    std::unique_lock<std::mutex> lock(mtx);
    for (auto& pose : pose_queue) {
      std::cout << "timestamp: " << pose.timestamp << ", data: " << pose.data << std::endl;
    }
  }

  bool findNearestPose(double rgb_timestamp, PoseType& pose, double& time_diff) {
    std::unique_lock<std::mutex> lock(mtx);
    if (pose_queue.empty()) {
      return false;
    }
    if (rgb_timestamp < pose_queue.front().timestamp || rgb_timestamp > pose_queue.back().timestamp) {
      return false;
    }

    auto it = std::lower_bound(pose_queue.begin(), pose_queue.end(), rgb_timestamp,
                               [](const TimestampedData<PoseType>& data, double ts) { return data.timestamp < ts; });
    if (it == pose_queue.end()) {
      pose = pose_queue.back().data;
      time_diff = std::abs(pose_queue.back().timestamp - rgb_timestamp);
    } else if (it == pose_queue.begin()) {
      pose = it->data;
      time_diff = std::abs(it->timestamp - rgb_timestamp);
    } else {
      auto prev = std::prev(it);
      double diff1 = std::abs(prev->timestamp - rgb_timestamp);
      double diff2 = std::abs(it->timestamp - rgb_timestamp);
      if (diff1 < diff2) {
        pose = prev->data;
        time_diff = diff1;
      } else {
        pose = it->data;
        time_diff = diff2;
      }
    }
    return true;
  }

  size_t size() {
    std::unique_lock<std::mutex> lock(mtx);
    return pose_queue.size();
  }

  void clear() {
    std::unique_lock<std::mutex> lock(mtx);
    pose_queue.clear();
  }
};

}  // namespace Utilities