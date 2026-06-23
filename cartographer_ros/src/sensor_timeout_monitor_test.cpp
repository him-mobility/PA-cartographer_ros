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

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace cartographer_ros {
namespace {

constexpr double kTimeout = 5.0;

TEST(SensorTimeoutMonitor, FreshSensorIsHealthy) {
  SensorTimeoutMonitor monitor(kTimeout);
  monitor.RecordData("imu", 100.0);
  EXPECT_FALSE(monitor.IsTimedOut("imu", 100.0));
  EXPECT_FALSE(monitor.IsTimedOut("imu", 104.0));
}

TEST(SensorTimeoutMonitor, SensorTimesOutAfterInterval) {
  SensorTimeoutMonitor monitor(kTimeout);
  monitor.RecordData("imu", 100.0);
  EXPECT_TRUE(monitor.IsTimedOut("imu", 106.0));
}

TEST(SensorTimeoutMonitor, ResumeClearsTimeout) {
  SensorTimeoutMonitor monitor(kTimeout);
  monitor.RecordData("imu", 100.0);
  ASSERT_TRUE(monitor.IsTimedOut("imu", 110.0));
  monitor.RecordData("imu", 110.0);  // data resumes
  EXPECT_FALSE(monitor.IsTimedOut("imu", 110.0));
}

TEST(SensorTimeoutMonitor, UnknownSensorIsNotReported) {
  SensorTimeoutMonitor monitor(kTimeout);
  EXPECT_FALSE(monitor.IsTimedOut("never", 100.0));
}

TEST(SensorTimeoutMonitor, TimedOutSensorsListsOnlyStaleOnes) {
  SensorTimeoutMonitor monitor(kTimeout);
  monitor.RecordData("imu", 100.0);
  monitor.RecordData("scan", 100.0);
  monitor.RecordData("scan", 106.0);  // scan stays fresh
  EXPECT_EQ(monitor.TimedOutSensors(106.0),
            (std::vector<std::string>{"imu"}));
}

}  // namespace
}  // namespace cartographer_ros
