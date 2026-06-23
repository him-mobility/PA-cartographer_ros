/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/sensor_timeout_monitor.h"

namespace cartographer_ros {

SensorTimeoutMonitor::SensorTimeoutMonitor(double timeout_seconds)
    : timeout_seconds_(timeout_seconds) {}

void SensorTimeoutMonitor::RecordData(const std::string& sensor_id,
                                      double now_seconds) {
  last_seen_seconds_[sensor_id] = now_seconds;
}

bool SensorTimeoutMonitor::IsTimedOut(const std::string& sensor_id,
                                      double now_seconds) const {
  const auto it = last_seen_seconds_.find(sensor_id);
  if (it == last_seen_seconds_.end()) {
    return false;
  }
  return now_seconds - it->second > timeout_seconds_;
}

std::vector<std::string> SensorTimeoutMonitor::TimedOutSensors(
    double now_seconds) const {
  std::vector<std::string> timed_out;
  for (const auto& entry : last_seen_seconds_) {
    if (now_seconds - entry.second > timeout_seconds_) {
      timed_out.push_back(entry.first);
    }
  }
  return timed_out;
}

}  // namespace cartographer_ros
