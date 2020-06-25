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
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <rosilo_conversions/rosilo_conversions.h>
#include <rosilo_clock/rosilo_clock.h>

//Eigen
#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace rosilo
{
class DensoCommunicationInterface{
private:
    bool enabled_;

    ros::Publisher publisher_target_joint_positions_;
    std_msgs::Float64MultiArray publisher_target_joint_positions_msg_;

    ros::Subscriber subscriber_joint_state_;
    ros::Subscriber subscriber_tool_pose_;

    VectorXd joint_positions_;

    DQ tool_pose_;

    void getJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void getToolPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    DensoCommunicationInterface() = delete;
    DensoCommunicationInterface(const DensoCommunicationInterface&) = delete;

    explicit DensoCommunicationInterface(const std::string& node_prefix, ros::NodeHandle& node_handle_publisher, ros::NodeHandle& node_handle_subscriber);

    explicit DensoCommunicationInterface(const std::string& node_prefix, ros::NodeHandle& node_handle);

    void set_target_joint_position(const VectorXd& target_positions);

    DQ get_tool_pose() const;

    VectorXd get_joint_positions() const;

    bool is_enabled() const;

};
}
