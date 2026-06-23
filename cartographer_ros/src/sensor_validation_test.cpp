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

#include <limits>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/imu.hpp"

namespace cartographer_ros {
namespace {

// An IMU message Cartographer accepts: covariance[0] != -1 (i.e. the sensor
// claims to provide the measurement) and all numeric fields are finite.
sensor_msgs::msg::Imu MakeValidImu() {
  sensor_msgs::msg::Imu imu;
  imu.linear_acceleration_covariance[0] = 0.0;
  imu.angular_velocity_covariance[0] = 0.0;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 9.81;
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  return imu;
}

TEST(SensorValidation, ValidImuIsValid) {
  EXPECT_TRUE(IsImuDataValid(MakeValidImu()));
}

TEST(SensorValidation, ImuWithoutLinearAccelerationIsInvalid) {
  sensor_msgs::msg::Imu imu = MakeValidImu();
  imu.linear_acceleration_covariance[0] = -1;
  EXPECT_FALSE(IsImuDataValid(imu));
}

TEST(SensorValidation, ImuWithoutAngularVelocityIsInvalid) {
  sensor_msgs::msg::Imu imu = MakeValidImu();
  imu.angular_velocity_covariance[0] = -1;
  EXPECT_FALSE(IsImuDataValid(imu));
}

TEST(SensorValidation, ImuWithNanLinearAccelerationIsInvalid) {
  sensor_msgs::msg::Imu imu = MakeValidImu();
  imu.linear_acceleration.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsImuDataValid(imu));
}

TEST(SensorValidation, ImuWithInfAngularVelocityIsInvalid) {
  sensor_msgs::msg::Imu imu = MakeValidImu();
  imu.angular_velocity.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(IsImuDataValid(imu));
}

}  // namespace
}  // namespace cartographer_ros
