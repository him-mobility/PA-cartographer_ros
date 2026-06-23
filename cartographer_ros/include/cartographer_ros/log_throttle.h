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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOG_THROTTLE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOG_THROTTLE_H

namespace cartographer_ros {

// Rate-limits a repeated log message: returns true on the first call and then
// at most once per 'interval_seconds'. Used to keep a continuously dropping
// sensor stream from flooding the log while still surfacing the problem to the
// developer. Not thread-safe.
class LogThrottle {
 public:
  explicit LogThrottle(double interval_seconds);

  // Production variant: uses a steady wall clock.
  bool ShouldLog();

  // Deterministic variant for tests: the caller supplies the current time.
  bool ShouldLogAt(double now_seconds);

 private:
  const double interval_seconds_;
  bool has_logged_ = false;
  double last_log_time_seconds_ = 0.0;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOG_THROTTLE_H
