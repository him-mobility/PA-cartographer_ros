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

#include "cartographer_ros/log_throttle.h"

#include <chrono>

namespace cartographer_ros {

LogThrottle::LogThrottle(double interval_seconds)
    : interval_seconds_(interval_seconds) {}

bool LogThrottle::ShouldLogAt(double now_seconds) {
  if (!has_logged_ ||
      now_seconds - last_log_time_seconds_ >= interval_seconds_) {
    has_logged_ = true;
    last_log_time_seconds_ = now_seconds;
    return true;
  }
  return false;
}

bool LogThrottle::ShouldLog() {
  const double now_seconds =
      std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  return ShouldLogAt(now_seconds);
}

}  // namespace cartographer_ros
