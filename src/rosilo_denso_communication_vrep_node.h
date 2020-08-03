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

#include <atomic>
#include <memory>

//ROS related
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

#include <rosilo_clock/rosilo_clock.h>
#include <rosilo_datalogger/rosilo_datalogger_interface.h>

using namespace Eigen;

namespace rosilo
{

struct DensoCommunicationVREPNodeInitialConfiguration
{
    int thread_sampling_time_nsec;

    bool enable_realtime_scheduling;
    int thread_estimated_computation_time_upper_bound_nsec;
    int thread_relative_deadline_nsec;

    std::string vrep_ip;
    int vrep_port;
    std::vector<std::string> vrep_joint_names;
};

class DensoCommunicationVREPNode
{
private:
    ///ROS
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

    std::string node_prefix_;

    ///Plotter
    std::unique_ptr<rosilo::DataloggerInterface> datalogger_;
    //Control loop time measurement
    std::unique_ptr<rosilo::Clock> clock_;
    //Vrep interface
    std::unique_ptr<DQ_VrepInterface> vrep_interface_;

    //Memory managemeny
    VectorXd target_joint_positions_;
    VectorXd joint_positions_;

    //Bool to kill loops
    std::atomic_bool* kill_this_node_;

    //Configuration
    DensoCommunicationVREPNodeInitialConfiguration configuration_;

    void updateTargetJointPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void publishJointStates(const VectorXd& joint_positions);

    bool should_shutdown();
    void shutdown();

public:
    DensoCommunicationVREPNode()=delete;
    DensoCommunicationVREPNode(const DensoCommunicationVREPNode&)=delete;

    //Constructor
    DensoCommunicationVREPNode(const DensoCommunicationVREPNodeInitialConfiguration& configuration, std::atomic_bool *kill_this_node);
    ~DensoCommunicationVREPNode()=default;

    int control_loop();
};

}

