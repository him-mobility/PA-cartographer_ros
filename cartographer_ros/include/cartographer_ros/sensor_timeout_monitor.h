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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_TIMEOUT_MONITOR_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_TIMEOUT_MONITOR_H

#include <map>
#include <string>
#include <vector>

namespace cartographer_ros {

// Tracks the last time each sensor delivered a message so that a silent sensor
// (disconnected, dead driver, stalled topic) can be surfaced to the developer.
// This complements per-message validation: validation drops bad values that DO
// arrive, whereas this detects the absence of messages entirely. Time is passed
// in by the caller so the logic is deterministically testable. Not thread-safe.
class SensorTimeoutMonitor {
 public:
  explicit SensorTimeoutMonitor(double timeout_seconds);

  // Registers that 'sensor_id' produced a message at 'now_seconds'. Also used
  // to register an expected sensor at startup so a never-arriving sensor is
  // detected.
  void RecordData(const std::string& sensor_id, double now_seconds);

  // True if the sensor is known and has not produced data within the timeout.
  bool IsTimedOut(const std::string& sensor_id, double now_seconds) const;

  // All known sensors that are currently timed out.
  std::vector<std::string> TimedOutSensors(double now_seconds) const;

 private:
  const double timeout_seconds_;
  std::map<std::string, double> last_seen_seconds_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_TIMEOUT_MONITOR_H
