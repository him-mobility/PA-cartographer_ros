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

#include "cartographer_ros/node_constants.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace cartographer_ros {
namespace {

TEST(NodeConstants, SingleTopicKeepsName) {
  EXPECT_EQ(ComputeRepeatedTopicNames("scan", 1),
            (std::vector<std::string>{"scan"}));
}

TEST(NodeConstants, MultipleTopicsAreNumbered) {
  EXPECT_EQ(ComputeRepeatedTopicNames("scan", 3),
            (std::vector<std::string>{"scan_1", "scan_2", "scan_3"}));
}

TEST(NodeConstants, NegativeTopicCountReturnsEmptyInsteadOfAborting) {
  EXPECT_TRUE(ComputeRepeatedTopicNames("scan", -1).empty());
}

TEST(NodeConstants, RecognizesPbstreamFilename) {
  EXPECT_TRUE(IsPbstreamFilename("map.pbstream"));
  EXPECT_TRUE(IsPbstreamFilename("/path/to/map.pbstream"));
}

TEST(NodeConstants, RejectsNonPbstreamFilename) {
  EXPECT_FALSE(IsPbstreamFilename("map.txt"));
  EXPECT_FALSE(IsPbstreamFilename("pbstream"));  // shorter than the suffix
  EXPECT_FALSE(IsPbstreamFilename(""));
}

}  // namespace
}  // namespace cartographer_ros
