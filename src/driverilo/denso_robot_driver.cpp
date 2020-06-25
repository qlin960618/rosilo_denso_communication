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

#include "denso_robot_driver.h"

namespace driverilo
{
const int DensoRobotDriver::SLAVE_MODE_JOINT_CONTROL        = 0x102;
const int DensoRobotDriver::SLAVE_MODE_END_EFFECTOR_CONTROL = 0x103;

DensoRobotDriver::DensoRobotDriver(std::string server_ip_address, const int server_port_number):
    bCapDriver_(server_ip_address, server_port_number)
{
    joint_positions_.resize(6);
    end_effector_pose_.resize(7);

    joint_positions_buffer_.resize(8,0);
    end_effector_pose_euler_buffer_.resize(7,0);
    end_effector_pose_homogenous_transformation_buffer_.resize(10,0);
}

std::tuple<VectorXd, bool> DensoRobotDriver::get_joint_positions()
{
    bool error_code = bCapDriver_.get_joint_positions(joint_positions_buffer_);
    Map<VectorXd> joint_positions_(joint_positions_buffer_.data(),6);
    return std::make_tuple(joint_positions_, error_code);
}

std::tuple<VectorXd, bool> DensoRobotDriver::get_end_effector_pose_homogenous_transformation()
{
    bool error_code = bCapDriver_.get_end_effector_pose_homogenous_transformation(end_effector_pose_homogenous_transformation_buffer_);
    Map<VectorXd> end_effetor_pose(end_effector_pose_homogenous_transformation_buffer_.data(),10);
    return std::make_tuple(end_effetor_pose, error_code);
}

double DensoRobotDriver::_sign(const double& a) const
{
    if(a>0)
        return 1;
    else
        return -1;
}

DQ DensoRobotDriver::_homogenous_vector_to_dq(const VectorXd& homogenousvector) const
{
    Vector3d translation;
    translation(0)= homogenousvector(0)/1000.0;
    translation(1)= homogenousvector(1)/1000.0;
    translation(2)= homogenousvector(2)/1000.0;
    Vector3d v1;
    v1(0) = homogenousvector(3);
    v1(1) = homogenousvector(4);
    v1(2) = homogenousvector(5);
    v1.normalize();
    Vector3d v2;
    v2(0) = homogenousvector(6);
    v2(1) = homogenousvector(7);
    v2(2) = homogenousvector(8);
    v2.normalize();
    Vector3d v0 = v1.cross(v2);
    Quaterniond q;

    const double& r11 = v0(0);
    const double& r21 = v0(1);
    const double& r31 = v0(2);

    const double& r12 = v1(0);
    const double& r22 = v1(1);
    const double& r32 = v1(2);

    const double& r13 = v2(0);
    const double& r23 = v2(1);
    const double& r33 = v2(2);

    q.w() = 0.5*sqrt(1+r11+r22+r33);
    q.x() = 0.5*_sign(r32-r23)*sqrt(r11-r22-r33+1);
    q.y() = 0.5*_sign(r13-r31)*sqrt(r22-r33-r11+1);
    q.z() = 0.5*_sign(r21-r12)*sqrt(r33-r11-r22+1);
    q.normalize();
    DQ rot(q.w(),q.x(),q.y(),q.z());
    DQ trans = 1+0.5*E_*(translation(0)*i_+translation(1)*j_+translation(2)*k_);
    DQ pose = trans*rot;
    return pose;
}

std::tuple<DQ, bool> DensoRobotDriver::get_end_effector_pose_dq()
{
    bool error_code = bCapDriver_.get_end_effector_pose_homogenous_transformation(end_effector_pose_homogenous_transformation_buffer_);
    Map<VectorXd> homogenousvector(end_effector_pose_homogenous_transformation_buffer_.data(),10);
    return std::make_tuple(_homogenous_vector_to_dq(homogenousvector), error_code);
}

bool DensoRobotDriver::set_joint_positions(const VectorXd &desired_joint_positions)
{
    std::vector<double> joint_positions_local_buffer(desired_joint_positions.data(), desired_joint_positions.data() + 6);
    return bCapDriver_.set_joint_positions(joint_positions_local_buffer);
}


std::tuple<VectorXd, bool> DensoRobotDriver::set_and_get_joint_positions(const VectorXd &desired_joint_positions)
{
    std::vector<double> set_joint_positions_local_buffer(desired_joint_positions.data(), desired_joint_positions.data() + 6);
    bool error_code = bCapDriver_.set_and_get_joint_positions(set_joint_positions_local_buffer, joint_positions_buffer_);
    Map<VectorXd> joint_positions(joint_positions_buffer_.data(),6);
    return std::make_tuple(joint_positions, error_code);
}

VectorXd DensoRobotDriver::_dq_to_homogenous_vector(const DQ& pose) const
{
    const double& n  = pose.q(0);
    const double& ex = pose.q(1);
    const double& ey = pose.q(2);
    const double& ez = pose.q(3);

    Vector3d v1;
    v1(0) = 2*(ex*ey-n*ez);
    v1(1) = 2*(n*n+ey*ey)-1;
    v1(2) = 2*(ey*ez+n*ex);

    Vector3d v2;
    v2(0) = 2*(ex*ez+n*ey);
    v2(1) = 2*(ey*ez-n*ex);
    v2(2) = 2*(n*n+ez*ez)-1;

    DQ trans = pose.translation();

    VectorXd homogenousvector(10);
    homogenousvector(0) = 1000.0*trans.q(1);
    homogenousvector(1) = 1000.0*trans.q(2);
    homogenousvector(2) = 1000.0*trans.q(3);

    v1 = v1.normalized();
    homogenousvector(3) = v1(0);
    homogenousvector(4) = v1(1);
    homogenousvector(5) = v1(2);

    v2 = v2.normalized();
    homogenousvector(6) = v2(0);
    homogenousvector(7) = v2(1);
    homogenousvector(8) = v2(2);

    //Figure selected automatically
    homogenousvector(9) = -1;

    return homogenousvector;
}

bool DensoRobotDriver::set_end_effector_pose_dq(const DQ& pose)
{
    VectorXd homogenousvector = _dq_to_homogenous_vector(pose);

    std::vector<double> end_effector_pose_local_buffer(homogenousvector.data(), homogenousvector.data() + 10);
    return bCapDriver_.set_end_effector_pose_homogenous_transformation(end_effector_pose_local_buffer);
}

void DensoRobotDriver::connect()
{
    bool worked; //bCap communication error code

    worked = bCapDriver_.open();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO Open() IN FUNCTION Connect(). Error code = " + bCapDriver_.get_last_error_info());
    }

    worked = bCapDriver_.service_start();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO ServiceStart() IN FUNCTION Connect(). Error code = " + bCapDriver_.get_last_error_info());
    }

    worked = bCapDriver_.controller_connect();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO ControllerConnect() IN FUNCTION Connect(). Error code = " + bCapDriver_.get_last_error_info());
    }

    bCapDriver_.initialize_controller_variable_handles();

    worked = bCapDriver_.get_robot();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO GetRobot() IN FUNCTION Connect(). Error code = " + bCapDriver_.get_last_error_info());
    }

    worked = bCapDriver_.take_arm();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO TakeArm() IN FUNCTION Connect(). Error code = " + bCapDriver_.get_last_error_info());
    }
}

void DensoRobotDriver::motor_on()
{
    bool worked; //bCap communication error code

    worked = bCapDriver_.set_motor_state(true);
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO SetMotorState() IN FUNCTION MotorOn(). Error code = " + bCapDriver_.get_last_error_info());
    }
}

void DensoRobotDriver::set_speed(const float& speed, const float& acceleration, const float& deacceleration)
{
    bool worked; //bCap communication error code

    worked = bCapDriver_.set_speed(speed,acceleration,deacceleration);
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO SetSpeed() IN FUNCTION SetSpeed(). Error code = " + bCapDriver_.get_last_error_info());
    }
}

void DensoRobotDriver::slave_mode_on(int mode)
{
    bool worked; //bCap communication error code

    worked = bCapDriver_.set_slave_mode(mode);
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO SlaveModeOn() IN FUNCTION SlaveModeOn(). Error code = " + bCapDriver_.get_last_error_info());
    }
}

void DensoRobotDriver::motor_off() noexcept
{
    bCapDriver_.set_motor_state(false);
}

void DensoRobotDriver::slave_mode_off() noexcept
{
    bCapDriver_.set_slave_mode(0);
}

void DensoRobotDriver::disconnect() noexcept
{
    bCapDriver_.give_arm();
    bCapDriver_.release_robot();
    bCapDriver_.controller_disconnect();
    bCapDriver_.service_stop();
    bCapDriver_.close();
}

}


