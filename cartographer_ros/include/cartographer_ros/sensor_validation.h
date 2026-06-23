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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_VALIDATION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_VALIDATION_H

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace cartographer_ros {

// Returns true if the IMU message is safe to forward to Cartographer, i.e. it
// claims to provide the required measurements and carries only finite values.
bool IsImuDataValid(const sensor_msgs::msg::Imu& imu);

// Returns true if the laser scan's header fields satisfy Cartographer's
// assumptions (non-negative range_min, range_max >= range_min). Individual NaN/
// Inf/out-of-range points are not a reason to reject the whole scan; they are
// filtered per-point (see CountUsableRanges / IsLaserScanUsable).
bool IsLaserScanValid(const sensor_msgs::msg::LaserScan& scan);

// Number of range readings that are finite and within [range_min, range_max].
int CountUsableRanges(const sensor_msgs::msg::LaserScan& scan);

// Returns true if the scan still carries at least one usable point after
// per-point filtering. An all-NaN/empty scan is dropped (returns false).
bool IsLaserScanUsable(const sensor_msgs::msg::LaserScan& scan);

// Returns true if the odometry pose (position and orientation) is finite.
bool IsOdometryValid(const nav_msgs::msg::Odometry& odometry);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_VALIDATION_H
