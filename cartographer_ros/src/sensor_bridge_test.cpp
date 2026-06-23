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

#include "cartographer_ros/sensor_bridge.h"

#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {
namespace {

namespace carto = ::cartographer;

// Records how many times each sensor-data sink was called so tests can assert
// whether a message was forwarded to Cartographer or dropped at the bridge.
class CountingTrajectoryBuilder
    : public carto::mapping::TrajectoryBuilderInterface {
 public:
  void AddSensorData(const std::string&,
                     const carto::sensor::TimedPointCloudData&) override {
    ++point_cloud_count;
  }
  void AddSensorData(const std::string&,
                     const carto::sensor::ImuData&) override {
    ++imu_count;
  }
  void AddSensorData(const std::string&,
                     const carto::sensor::OdometryData&) override {
    ++odometry_count;
  }
  void AddSensorData(const std::string&,
                     const carto::sensor::FixedFramePoseData&) override {}
  void AddSensorData(const std::string&,
                     const carto::sensor::LandmarkData&) override {}
  void AddLocalSlamResultData(
      std::unique_ptr<carto::mapping::LocalSlamResultData>) override {}

  int point_cloud_count = 0;
  int imu_count = 0;
  int odometry_count = 0;
};

class SensorBridgeTest : public ::testing::Test {
 protected:
  SensorBridgeTest()
      : clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
        tf_buffer_(clock_) {
    // IMU is required to be colocated with the tracking frame, so use identity.
    geometry_msgs::msg::TransformStamped imu_to_tracking;
    imu_to_tracking.header.frame_id = "tracking";
    imu_to_tracking.child_frame_id = "imu";
    imu_to_tracking.transform.rotation.w = 1.0;
    tf_buffer_.setTransform(imu_to_tracking, "test", /*is_static=*/true);

    geometry_msgs::msg::TransformStamped odom_to_tracking;
    odom_to_tracking.header.frame_id = "tracking";
    odom_to_tracking.child_frame_id = "odom_child";
    odom_to_tracking.transform.rotation.w = 1.0;
    tf_buffer_.setTransform(odom_to_tracking, "test", /*is_static=*/true);
  }

  std::unique_ptr<SensorBridge> MakeBridge() {
    return std::make_unique<SensorBridge>(
        /*num_subdivisions_per_laser_scan=*/1, "tracking",
        /*lookup_transform_timeout_sec=*/0.1, &tf_buffer_, &builder_);
  }

  static sensor_msgs::msg::Imu::SharedPtr MakeValidImu() {
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->header.frame_id = "imu";
    imu->header.stamp.sec = 1;
    imu->linear_acceleration_covariance[0] = 0.0;
    imu->angular_velocity_covariance[0] = 0.0;
    imu->linear_acceleration.z = 9.81;
    return imu;
  }

  static nav_msgs::msg::Odometry::SharedPtr MakeValidOdometry() {
    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->header.frame_id = "odom";
    odometry->header.stamp.sec = 1;
    odometry->child_frame_id = "odom_child";
    odometry->pose.pose.orientation.w = 1.0;
    return odometry;
  }

  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf_buffer_;
  CountingTrajectoryBuilder builder_;
};

TEST_F(SensorBridgeTest, ForwardsValidImu) {
  MakeBridge()->HandleImuMessage("imu", MakeValidImu());
  EXPECT_EQ(builder_.imu_count, 1);
}

TEST_F(SensorBridgeTest, DropsImuWithoutLinearAcceleration) {
  auto imu = MakeValidImu();
  imu->linear_acceleration_covariance[0] = -1;
  MakeBridge()->HandleImuMessage("imu", imu);
  EXPECT_EQ(builder_.imu_count, 0);
}

TEST_F(SensorBridgeTest, DropsImuWithNanValues) {
  auto imu = MakeValidImu();
  imu->angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  MakeBridge()->HandleImuMessage("imu", imu);
  EXPECT_EQ(builder_.imu_count, 0);
}

TEST_F(SensorBridgeTest, ForwardsValidOdometry) {
  MakeBridge()->HandleOdometryMessage("odometry", MakeValidOdometry());
  EXPECT_EQ(builder_.odometry_count, 1);
}

TEST_F(SensorBridgeTest, DropsOdometryWithNanPose) {
  auto odometry = MakeValidOdometry();
  odometry->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  MakeBridge()->HandleOdometryMessage("odometry", odometry);
  EXPECT_EQ(builder_.odometry_count, 0);
}

}  // namespace
}  // namespace cartographer_ros
