#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_denso_communcation.
#
#    rosilo_denso_communcation is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_denso_communcation is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_denso_communcation.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include <exception>
#include <tuple>

#include <dqrobotics/DQ.h>

#include "bcap_driver.h"

using namespace DQ_robotics;
using namespace Eigen;

namespace driverilo
{
class DensoRobotDriver
{

private:
    //BCAP driver
    BCAPDriver bCapDriver_;

    //Joint positions
    VectorXd joint_positions_;
    VectorXd end_effector_pose_;
    std::vector<double> joint_positions_buffer_;
    std::vector<double> end_effector_pose_euler_buffer_;
    std::vector<double> end_effector_pose_homogenous_transformation_buffer_;

    //Internal functions
    DQ _homogenous_vector_to_dq(const VectorXd& homogenousvector) const;
    VectorXd _dq_to_homogenous_vector(const DQ& pose) const;
    double _sign(const double &a) const;

public:
    //Constants
    const static int SLAVE_MODE_JOINT_CONTROL;
    const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    //Make this class non-copyable
    DensoRobotDriver(const DensoRobotDriver&)=delete;
    //Remove the default constructor
    DensoRobotDriver()=delete;

    DensoRobotDriver(std::string server_ip_address, const int server_port_number);

    std::tuple<VectorXd, bool> get_joint_positions();
    std::tuple<VectorXd, bool> get_end_effector_pose_homogenous_transformation();
    std::tuple<DQ, bool> get_end_effector_pose_dq();

    bool set_joint_positions(const VectorXd& desired_joint_positions);
    std::tuple<VectorXd, bool> set_and_get_joint_positions(const VectorXd& desired_joint_positions);
    bool set_end_effector_pose_dq(const DQ& pose);

    void connect(); //Throws std::runtime_error()
    void disconnect() noexcept; //No exceptions should be thrown in the path to turn off the robot

    void motor_on(); //Throws std::runtime_error()
    void motor_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

    void set_speed(const float& speed, const float& acceleration, const float& deacceleration); //Throws std::runtime_error()

    void slave_mode_on(int mode); //Throws std::runtime_error()
    void slave_mode_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

};
}



