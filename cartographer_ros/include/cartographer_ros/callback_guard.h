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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_CALLBACK_GUARD_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_CALLBACK_GUARD_H

#include <exception>
#include <string>

#include "cartographer_ros/log_throttle.h"
#include "glog/logging.h"

namespace cartographer_ros {

// Runs 'callback', swallowing any exception so that a single faulty message or
// transient error cannot tear down the whole node (an uncaught exception would
// otherwise propagate out of the executor and terminate the process). The
// failure is logged, rate-limited by 'throttle'. Returns true if the callback
// ran without throwing.
//
// Note: this only catches C++ exceptions. glog CHECK failures call abort() and
// cannot be caught here -- those are prevented by validating data up front.
template <typename Callback>
bool RunGuarded(const std::string& context, LogThrottle& throttle,
                Callback&& callback) {
  try {
    callback();
    return true;
  } catch (const std::exception& e) {
    if (throttle.ShouldLog()) {
      LOG(ERROR) << "Exception in " << context << ": " << e.what()
                 << ". Ignoring this invocation so the node keeps running.";
    }
    return false;
  } catch (...) {
    if (throttle.ShouldLog()) {
      LOG(ERROR) << "Unknown exception in " << context
                 << ". Ignoring this invocation so the node keeps running.";
    }
    return false;
  }
}

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_CALLBACK_GUARD_H
