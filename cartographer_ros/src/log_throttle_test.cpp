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

#include "gtest/gtest.h"

namespace cartographer_ros {
namespace {

TEST(LogThrottle, LogsOnFirstUse) {
  LogThrottle throttle(5.0);
  EXPECT_TRUE(throttle.ShouldLogAt(100.0));
}

TEST(LogThrottle, SuppressesWithinInterval) {
  LogThrottle throttle(5.0);
  throttle.ShouldLogAt(100.0);
  EXPECT_FALSE(throttle.ShouldLogAt(102.0));
}

TEST(LogThrottle, LogsAgainAfterInterval) {
  LogThrottle throttle(5.0);
  throttle.ShouldLogAt(100.0);
  EXPECT_TRUE(throttle.ShouldLogAt(105.0));
}

TEST(LogThrottle, LogsOnlyOncePerBurst) {
  LogThrottle throttle(5.0);
  int logged = 0;
  for (int i = 0; i < 100; ++i) {
    if (throttle.ShouldLogAt(100.0 + i * 0.01)) ++logged;
  }
  EXPECT_EQ(logged, 1);
}

}  // namespace
}  // namespace cartographer_ros
