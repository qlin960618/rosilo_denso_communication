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

//ROS related
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <rosilo_clock/rosilo_clock.h>
#include <rosilo_datalogger/rosilo_datalogger_interface.h>

using namespace Eigen;

namespace rosilo
{
class DensoCommunicationDummyNode
{
private:
    //ROS
    ros::NodeHandle    subscriber_node_handle_;
    ros::CallbackQueue subscriber_callback_queue_;
    ros::Subscriber    subscriber_target_joint_positions_;
    ros::Subscriber    subscriber_target_tool_pose_;

    ros::NodeHandle    publisher_node_handle_;
    ros::CallbackQueue publisher_callback_queue_;
    ros::Publisher     publisher_joint_state_;
    sensor_msgs::JointState publisher_joint_state_msg_;

    ros::NodeHandle    datalogger_node_handle_;
    ros::CallbackQueue datalogger_callback_queue_;
    rosilo::DataloggerInterface* datalogger_;

    //Memory managemeny
    VectorXd target_joint_positions_;
    VectorXd joint_positions_;

    //Control loop time measurement
    rosilo::Clock* clock_;
    int    thread_sampling_time_nsec_;
    double thread_sampling_time_nsec_d_;
    double thread_sampling_time_sec_d_;
    int    thread_estimated_computation_time_upper_bound_nsec_;
    int    thread_relative_deadline_nsec_;

    //Bool to kill loops
    bool* kill_this_node_;

    std::string node_prefix_;
    bool realtime_scheduling_enabled_;

    void updateTargetJointPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void publishJointStates(const VectorXd& joint_positions);

    DensoCommunicationDummyNode(const DensoCommunicationDummyNode&)=delete;

    bool shouldShutdown();
    void shutdown();

public:

    //Constructor
    DensoCommunicationDummyNode(const int thread_sampling_time_nsec, const int thread_estimated_computation_time_upper_bound_nsec, const int thread_relative_deadline_nsec, const bool enable_realtime_scheduling, bool* kill_this_node);
    ~DensoCommunicationDummyNode();

    int control_loop();
};
}


