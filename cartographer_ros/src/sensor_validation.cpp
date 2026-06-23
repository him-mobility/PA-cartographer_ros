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

#include "cartographer_ros/sensor_validation.h"

#include <cmath>

namespace cartographer_ros {
namespace {

bool IsFinite(const geometry_msgs::msg::Vector3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

bool IsImuDataValid(const sensor_msgs::msg::Imu& imu) {
  // A covariance[0] of -1 is the ROS convention for "this measurement is not
  // provided"; Cartographer requires both measurements to operate.
  if (imu.linear_acceleration_covariance[0] == -1) return false;
  if (imu.angular_velocity_covariance[0] == -1) return false;
  if (!IsFinite(imu.linear_acceleration)) return false;
  if (!IsFinite(imu.angular_velocity)) return false;
  return true;
}

bool IsLaserScanValid(const sensor_msgs::msg::LaserScan& scan) {
  if (!(scan.range_min >= 0.f)) return false;
  if (!(scan.range_max >= scan.range_min)) return false;
  return true;
}

int CountUsableRanges(const sensor_msgs::msg::LaserScan& scan) {
  int count = 0;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= scan.range_min &&
        range <= scan.range_max) {
      ++count;
    }
  }
  return count;
}

bool IsLaserScanUsable(const sensor_msgs::msg::LaserScan& scan) {
  return CountUsableRanges(scan) > 0;
}

}  // namespace cartographer_ros
