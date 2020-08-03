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

#include "rosilo_denso_communication/rosilo_denso_communication_interface.h"

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

void DensoCommunicationInterface::_get_joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(!enabled_)
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Initializing DensoCommunicationInterface enabled.");
        enabled_=true;
    }
    joint_positions_ = rosilo::std_vector_double_to_vectorxd(msg->position);
}

void DensoCommunicationInterface::_get_tool_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tool_pose_ = rosilo::geometry_msgs_pose_stamped_to_dq(*msg);
}
DensoCommunicationInterface::DensoCommunicationInterface(const std::string& node_prefix, ros::NodeHandle& node_handle_publisher, ros::NodeHandle& node_handle_subscriber):
    enabled_(false),
    joint_positions_(VectorXd::Zero(6)),
    tool_pose_(1)
{
    ROS_INFO_STREAM(ros::this_node::getName()+"::Initializing DensoCommunicationInterface with prefix " + node_prefix);
    subscriber_joint_state_           = node_handle_subscriber.subscribe(node_prefix+"get/joint_state", 1, &DensoCommunicationInterface::_get_joint_state_callback, this);
    subscriber_tool_pose_             = node_handle_subscriber.subscribe(node_prefix+"get/tool_pose",1,&DensoCommunicationInterface::_get_tool_pose_callback,this);

    publisher_target_joint_positions_ = node_handle_publisher.advertise<std_msgs::Float64MultiArray>(node_prefix+"set/target_joint_positions",1);

    publisher_target_joint_positions_msg_.data.resize(6,0);
}

DensoCommunicationInterface::DensoCommunicationInterface(const std::string& node_prefix, ros::NodeHandle& node_handle):
    enabled_(false),
    joint_positions_(VectorXd::Zero(6)),
    tool_pose_(1)
{
    ROS_INFO_STREAM(ros::this_node::getName()+"::Initializing DensoCommunicationInterface with prefix " + node_prefix);
    subscriber_joint_state_           = node_handle.subscribe(node_prefix+"get/joint_state", 1, &DensoCommunicationInterface::_get_joint_state_callback, this);
    subscriber_tool_pose_             = node_handle.subscribe(node_prefix+"get/tool_pose",   1, &DensoCommunicationInterface::_get_tool_pose_callback,   this);

    publisher_target_joint_positions_ = node_handle.advertise<std_msgs::Float64MultiArray>(node_prefix+"set/target_joint_positions",1);

    publisher_target_joint_positions_msg_.data.resize(6,0);
}

void DensoCommunicationInterface::set_target_joint_position(const VectorXd& target_positions)
{
    publisher_target_joint_positions_msg_.data = rosilo::vectorxd_to_std_vector_double(target_positions);
    publisher_target_joint_positions_.publish(publisher_target_joint_positions_msg_);
}

DQ DensoCommunicationInterface::get_tool_pose() const
{
    return tool_pose_;
}

VectorXd DensoCommunicationInterface::get_joint_positions() const
{
    return joint_positions_;
}

bool DensoCommunicationInterface::is_enabled() const
{
    return enabled_;
}



}

