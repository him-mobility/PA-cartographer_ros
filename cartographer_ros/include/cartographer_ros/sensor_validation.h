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

#include "sensor_msgs/msg/imu.hpp"

namespace cartographer_ros {

// Returns true if the IMU message is safe to forward to Cartographer, i.e. it
// claims to provide the required measurements and carries only finite values.
bool IsImuDataValid(const sensor_msgs::msg::Imu& imu);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_VALIDATION_H
