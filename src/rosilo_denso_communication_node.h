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

#include <memory>
#include <atomic>

//ROS related
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <rosilo_bcap_driver/rosilo_denso_robot_driver.h>
#include <rosilo_clock/rosilo_clock.h>
#include <rosilo_datalogger/rosilo_datalogger_interface.h>

using namespace Eigen;

namespace rosilo
{

class DensoCommunication
{
private:
    //ROS
    ros::NodeHandle    subscriber_node_handle_;
    ros::CallbackQueue subscriber_callback_queue_;
    ros::Subscriber    subscriber_target_joint_positions_;

    ros::NodeHandle    publisher_node_handle_;
    ros::CallbackQueue publisher_callback_queue_;
    ros::Publisher     publisher_joint_state_;
    sensor_msgs::JointState publisher_joint_state_msg_;
    ros::Publisher     publisher_tool_pose_;
    geometry_msgs::PoseStamped publisher_tool_pose_msg_;

    ros::NodeHandle    datalogger_node_handle_;
    ros::CallbackQueue datalogger_callback_queue_;
    std::unique_ptr<rosilo::DataloggerInterface> datalogger_;

    //Memory managemeny
    VectorXd target_joint_positions_;
    VectorXd last_joint_positions_;
    VectorXd joint_positions_;
    VectorXd joint_velocities_;

    DQ tool_pose_;

    //
    double speed_;

    //Control loop time measurement
    std::unique_ptr<rosilo::Clock> clock_;
    int    thread_sampling_time_nsec_;
    double thread_sampling_time_nsec_d_;
    double thread_sampling_time_sec_d_;
    int    thread_estimated_computation_time_upper_bound_nsec_;
    int    thread_relative_deadline_nsec_;

    //Bool to kill loops
    std::atomic_bool* kill_this_node_;
    bool read_only_;

    //Denso comm
    std::unique_ptr<rosilo::DensoRobotDriver> robot_;
    bool robot_communication_ok_;

    std::string node_prefix_;
    bool realtime_scheduling_enabled_;

    void updateTargetJointPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void publishJointStates(const VectorXd& joint_positions, const Eigen::VectorXd &joint_velocities);
    void publishToolPose(const DQ& tool_pose);

    void connect();
    void disconnect();
    void motorOn();
    void motorOff();

    void shutdown();

public:
    DensoCommunication(const DensoCommunication&)=delete;
    DensoCommunication()=delete;

    //Constructor
    DensoCommunication(const std::string& robot_ip_address, const int& port, const int thread_sampling_time_nsec, const int thread_estimated_computation_time_upper_bound_nsec, const int thread_relative_deadline_nsec, const bool enable_realtime_scheduling, bool read_only, const double& speed, std::atomic_bool* kill_this_node);
    ~DensoCommunication()=default;

    int control_loop();
};

}

