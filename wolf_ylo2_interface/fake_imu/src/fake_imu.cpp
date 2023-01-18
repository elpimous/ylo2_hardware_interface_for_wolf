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

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace {

constexpr double kFakeGravity = 9.8;
constexpr int kSubscriberQueueSize = 150;
constexpr char kImuInTopic[] = "imu_in";
constexpr char kImuOutTopic[] = "imu/data";

}  // namespace


#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv){

  ::ros::init(argc, argv, "fake_imu");

  ::ros::NodeHandle node_handle;
  ::ros::Publisher publisher = node_handle.advertise<sensor_msgs::Imu>(kImuOutTopic, 10);

  ::ros::Rate loop_rate(10);

  sensor_msgs::Imu imu_out;
  // TODO(damonkohler): This relies on the z-axis alignment of the
  // IMU with the Kobuki base.
  imu_out.header.frame_id = "imu_link";

  imu_out.linear_acceleration.x = 0.00000000001;
  imu_out.linear_acceleration.y = 0.00000000001;
  imu_out.linear_acceleration.z = kFakeGravity;

  imu_out.angular_velocity.x  = 0.0000000001;
  imu_out.angular_velocity.y  = 0.0000000001;
  imu_out.angular_velocity.z  = 0.0000000001;

  imu_out.orientation.x = 0.0000000001;
  imu_out.orientation.y = 0.0000000001;
  imu_out.orientation.z = 0.0000000001;

  while (ros::ok())  {
    imu_out.header.stamp = ros::Time::now();
    publisher.publish(imu_out);
    ::ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
