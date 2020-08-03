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

#include "rosilo_denso_communication_vrep_node.h"

#include <stdexcept>
#include <atomic>

#include <rosilo_conversions/eigen3_std_conversions.h>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void SigIntHandler(int)
{
    kill_this_process = true;
}

/*********************************************
 * GLOBAL SCOPE FUNCTIONS (INCLUDING MAIN)
 * *******************************************/
rosilo::DensoCommunicationVREPNode* create_instance_from_ros_parameter_server()
{
    ros::NodeHandle nodehandle;

    rosilo::DensoCommunicationVREPNodeInitialConfiguration configuration;

    ROS_INFO_STREAM("Trying to load Denso Communication VREP Node parameters for node " << ros::this_node::getName());
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_sampling_time_nsec",configuration.thread_sampling_time_nsec)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_estimated_computation_time_upper_bound_nsec",configuration.thread_estimated_computation_time_upper_bound_nsec)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/enable_real_time_scheduling",configuration.enable_realtime_scheduling)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_relative_deadline_nsec",configuration.thread_relative_deadline_nsec)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_ip",configuration.vrep_ip)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_port",configuration.vrep_port)){return nullptr;}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_joint_names",configuration.vrep_joint_names)){return nullptr;}

    return new rosilo::DensoCommunicationVREPNode(configuration, &kill_this_process);
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, SigIntHandler) == SIG_ERR)
    {
        std::cout << "Error setting the signal int handler." << std::endl;
        return 1;
    }

    ros::init(argc, argv,"arm2",ros::init_options::NoSigintHandler);

    rosilo::DensoCommunicationVREPNode* dc = create_instance_from_ros_parameter_server();
    if(dc != nullptr)
    {
        dc->control_loop();
        delete dc;
    }
    else
    {
        ROS_ERROR_STREAM("Unable to read parameter from parameter server.");
    }

    return 0;
}

namespace rosilo
{
/*********************************************
 * CLASS METHODS
 * *******************************************/

void DensoCommunicationVREPNode::updateTargetJointPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    target_joint_positions_ = rosilo::std_vector_double_to_vectorxd(msg->data);
}

void DensoCommunicationVREPNode::publishJointStates(const VectorXd& joint_positions)
{
    publisher_joint_state_msg_.position = std::vector<double>(joint_positions.data(),joint_positions.data()+6);
    publisher_joint_state_.publish(publisher_joint_state_msg_);
}

//Constructor
DensoCommunicationVREPNode::DensoCommunicationVREPNode(const DensoCommunicationVREPNodeInitialConfiguration& configuration, std::atomic_bool *kill_this_node)
{
    configuration_ = configuration;
    kill_this_node_ = kill_this_node;

    node_prefix_ = ros::this_node::getName();

    subscriber_node_handle_.setCallbackQueue(&subscriber_callback_queue_);
    subscriber_target_joint_positions_ = subscriber_node_handle_.subscribe(node_prefix_+"/set/target_joint_positions", 1, &DensoCommunicationVREPNode::updateTargetJointPositionsCallback, this);

    publisher_node_handle_.setCallbackQueue(&publisher_callback_queue_);
    publisher_joint_state_ = publisher_node_handle_.advertise<sensor_msgs::JointState>(node_prefix_+"/get/joint_state",1);

    datalogger_node_handle_.setCallbackQueue(&datalogger_callback_queue_);
    datalogger_ = std::unique_ptr<rosilo::DataloggerInterface>(new rosilo::DataloggerInterface(datalogger_node_handle_,10));

    //Initialize vectors
    target_joint_positions_  = VectorXd::Zero(6);
    joint_positions_         = VectorXd::Zero(6);

    //Initialize ros messages
    publisher_joint_state_msg_.effort.resize(6,0.0);
    publisher_joint_state_msg_.position.resize(6,0.0);
    publisher_joint_state_msg_.velocity.resize(6,0.0);

    clock_  = std::unique_ptr<rosilo::Clock>(new rosilo::Clock(configuration.thread_sampling_time_nsec));

    vrep_interface_ = std::unique_ptr<DQ_VrepInterface>(new DQ_VrepInterface(kill_this_node));
}

int DensoCommunicationVREPNode::control_loop()
{
    try
    {
        clock_->init();

        if(!vrep_interface_->connect(configuration_.vrep_ip,configuration_.vrep_port,10,100))
        {
            throw std::runtime_error(ros::this_node::getName() + "::Unable to connect to VREP at " + configuration_.vrep_ip);
        }
        ROS_INFO_STREAM(ros::this_node::getName() + "::Connected to VREP.");

        joint_positions_ = vrep_interface_->get_joint_positions(configuration_.vrep_joint_names);
        target_joint_positions_ = joint_positions_;

        ROS_INFO_STREAM(ros::this_node::getName() + "::Starting control loop...");
        while(not should_shutdown())
        {
            //Sleep
            clock_->update_and_sleep();

            subscriber_callback_queue_.callAvailable();
            vrep_interface_->set_joint_positions(configuration_.vrep_joint_names,target_joint_positions_);
            joint_positions_ = target_joint_positions_;

            publishJointStates(joint_positions_);
            publisher_callback_queue_.callAvailable();

            //Send data to datalogger
            datalogger_->log(node_prefix_.substr(1)+"_joint_positions",joint_positions_);
            datalogger_->log(node_prefix_.substr(1)+"_target_joint_positions",target_joint_positions_);
            datalogger_->log(node_prefix_.substr(1)+"_computational_time",clock_->get_computation_time());
            datalogger_->log(node_prefix_.substr(1)+"_desired_sampling_time",clock_->get_desired_thread_sampling_time_sec());
            datalogger_callback_queue_.callAvailable();

        }//End while not kill this node
        ROS_INFO_STREAM(ros::this_node::getName() + "::Control loop ended.");
        vrep_interface_->disconnect();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception caught::" << e.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Unknown error.");
    }

    return 0;
}//End function


bool DensoCommunicationVREPNode::should_shutdown()
{
    return (*kill_this_node_);
}

void DensoCommunicationVREPNode::shutdown()
{
    (*kill_this_node_) = true;
}
}
