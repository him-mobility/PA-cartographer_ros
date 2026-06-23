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

#include "cartographer_ros/callback_guard.h"

#include <stdexcept>

#include "gtest/gtest.h"

namespace cartographer_ros {
namespace {

TEST(CallbackGuard, RunsCallbackAndReturnsTrue) {
  LogThrottle throttle(5.0);
  bool ran = false;
  EXPECT_TRUE(RunGuarded("ctx", throttle, [&] { ran = true; }));
  EXPECT_TRUE(ran);
}

TEST(CallbackGuard, SwallowsStdExceptionAndReturnsFalse) {
  LogThrottle throttle(5.0);
  bool returned = true;
  EXPECT_NO_THROW({
    returned = RunGuarded("ctx", throttle,
                          [] { throw std::runtime_error("boom"); });
  });
  EXPECT_FALSE(returned);
}

TEST(CallbackGuard, SwallowsNonStandardException) {
  LogThrottle throttle(5.0);
  EXPECT_NO_THROW({ RunGuarded("ctx", throttle, [] { throw 42; }); });
}

TEST(CallbackGuard, ContinuesAfterException) {
  LogThrottle throttle(5.0);
  RunGuarded("ctx", throttle, [] { throw std::runtime_error("boom"); });
  bool ran = false;
  RunGuarded("ctx", throttle, [&] { ran = true; });
  EXPECT_TRUE(ran);
}

}  // namespace
}  // namespace cartographer_ros
