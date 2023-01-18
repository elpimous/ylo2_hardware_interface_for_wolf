/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/
// modified for ylo2 robot, Vincent FOUCAULT, elpimous12@mail.com.

#include "wolf_ylo2_interface/ylo2_robot_hw.hpp"
#include <chrono>
#include <thread>

namespace ylo22ros
{
using namespace hardware_interface;

int64_t utime_now() {
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
    throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
  uint32_t sec	= timeofday.tv_sec;
  uint32_t nsec = timeofday.tv_usec * 1000;
  return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

ylo2RobotHw::ylo2RobotHw(){
  robot_name_ = "ylo2";
}

ylo2RobotHw::~ylo2RobotHw(){}


//YloTwoPcanToMoteus command; // instance of class YloTwoPcanToMoteus


void ylo2RobotHw::init(const ros::NodeHandle& nh)
{
  // Hardware interfaces: Joints
  auto joint_names = loadJointNamesFromSRDF();
  if(joint_names.size()>0)
  {
    WolfRobotHwInterface::initializeJointsInterface(joint_names);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_effort_interface_);
  }
  else
  {
    ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
    return;
  }

  // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }
  ylo2RobotHw::startup_routine();
}


void ylo2RobotHw::startup_routine()
{
  //command.peak_fdcan_board_initialization();
  usleep(200);
  //command.check_initial_ground_pose();
  std::cout << "startup_routine Done." << std::endl;
  usleep(200);
}

void ylo2RobotHw::read()
{
  for (unsigned int jj = 0; jj < n_dof_; ++jj) // jj++
  {
    // Reset values
    float RX_pos = 0.0;
    float RX_vel = 0.0;
    float RX_tor = 0.0;
    float RX_volt = 0.0;
    float RX_temp = 0.0;
    float RX_fault = 0.0;

    //auto ids  = command.motor_adapters_[jj].getIdx();
    //int port  = command.motor_adapters_[jj].getPort();
    //auto sign = command.motor_adapters_[jj].getSign();

    //command.read_moteus_RX_queue(ids, port, RX_pos, RX_vel, RX_tor, RX_volt, RX_temp, RX_fault);  // query values;
    joint_position_[jj] = static_cast<double>((RX_pos*2*M_PI)); // joint turns to radians
    joint_velocity_[jj] = static_cast<double>(RX_vel);   // measured in revolutions / s
    joint_effort_[jj]   = static_cast<double>(RX_tor);   // measured in N*m
    usleep(200);
  }

  // IMU
  // Publish the IMU data NOTE: missing covariances
  /*if(imu_pub_.get() && imu_pub_->trylock())
  {
    imu_pub_->msg_.orientation.w         = static_cast<double>(0.000000000001);
    imu_pub_->msg_.orientation.x         = static_cast<double>(0.000000000001);
    imu_pub_->msg_.orientation.y         = static_cast<double>(0.000000000001);
    imu_pub_->msg_.orientation.z         = static_cast<double>(0.000000000001);

    imu_pub_->msg_.angular_velocity.x    = static_cast<double>(0.000000000001);
    imu_pub_->msg_.angular_velocity.y    = static_cast<double>(0.000000000001);
    imu_pub_->msg_.angular_velocity.z    = static_cast<double>(0.000000000001);

    imu_pub_->msg_.linear_acceleration.x = static_cast<double>(0.000000000001);
    imu_pub_->msg_.linear_acceleration.y = static_cast<double>(0.000000000001);
    imu_pub_->msg_.linear_acceleration.z = static_cast<double>(0.000000000001);
    
    imu_pub_->msg_.header.stamp = ros::Time::now();
    imu_pub_->unlockAndPublish();
  }
  */
}

void ylo2RobotHw::write(){
  for (unsigned int jj = 0; jj < n_dof_; ++jj){
      //auto ids  = command.motor_adapters_[jj].getIdx();
      //auto sign = command.motor_adapters_[jj].getSign();
      //int port  = command.motor_adapters_[jj].getPort();
      //command.send_moteus_TX_frame(ids, port, sign*static_cast<float>(joint_effort_command_[jj])); 
      usleep(120);
  }
  //command.send_power_board_order();
}

} // namespace
