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

#ifndef YLO2_ROBOT_HW_H
#define YLO2_ROBOT_HW_H

#include <wolf_hardware_interface/wolf_robot_hw.h>
#include "moteus_driver/YloTwoPcanToMoteus.hpp" // ylo2 library
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ylo22ros{

class ylo2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface{

    public:
        ylo2RobotHw();
        virtual ~ylo2RobotHw();
        void init(const ros::NodeHandle &nh);
        void read();
        void write();
    private:
        /** @brief IMU realtime publisher */
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;

        /** @brief IMU realtime publisher */
        //std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;

        /** @brief Sends a zero command to the robot */
        void send_zero_command();

        /** @brief Executes the robot's startup routine */
        void startup_routine();
        std::mutex startup_routine_mutex;
};

} // end namespace
#endif