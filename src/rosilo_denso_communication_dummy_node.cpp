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

#include "rosilo_denso_communication_dummy_node.h"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
bool kill_this_process = false;
void SigIntHandler(int)
{
    kill_this_process = true;
    //ROS_INFO_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

/*********************************************
 * GLOBAL SCOPE FUNCTIONS (INCLUDING MAIN)
 * *******************************************/
rosilo::DensoCommunicationDummyNode* create_instance_from_ros_parameter_server()
{
    ros::NodeHandle nodehandle;
    int thread_sampling_time_nsec;
    int thread_estimated_computation_time_upper_bound_nsec;
    int thread_relative_deadline_nsec;
    bool enable_real_time_scheduling;
    ROS_INFO_STREAM("Trying to load Denso Communication Dummy Node parameters for node " << ros::this_node::getName());
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_sampling_time_nsec",thread_sampling_time_nsec)){return nullptr;};
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_estimated_computation_time_upper_bound_nsec",thread_estimated_computation_time_upper_bound_nsec)){return nullptr;};
    if(!nodehandle.getParam(ros::this_node::getName()+"/enable_real_time_scheduling",enable_real_time_scheduling)){return nullptr;};
    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_relative_deadline_nsec",thread_relative_deadline_nsec)){return nullptr;};


    return new rosilo::DensoCommunicationDummyNode(thread_sampling_time_nsec,thread_estimated_computation_time_upper_bound_nsec,thread_relative_deadline_nsec,enable_real_time_scheduling,&kill_this_process);
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, SigIntHandler) == SIG_ERR)
    {
        std::cout << "Error setting the signal int handler." << std::endl;
        return 1;
    }

    ros::init(argc, argv,"uninitialized_denso_communication_node",ros::init_options::NoSigintHandler);

    rosilo::DensoCommunicationDummyNode* dc = create_instance_from_ros_parameter_server();
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

void DensoCommunicationDummyNode::updateTargetJointPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    target_joint_positions_ = VectorXd::Map(&msg->data[0], msg->data.size());
}

void DensoCommunicationDummyNode::publishJointStates(const VectorXd& joint_positions)
{
    publisher_joint_state_msg_.position = std::vector<double>(joint_positions.data(),joint_positions.data()+6);
    publisher_joint_state_.publish(publisher_joint_state_msg_);
}

//Constructor
DensoCommunicationDummyNode::DensoCommunicationDummyNode(const int thread_sampling_time_nsec, const int thread_estimated_computation_time_upper_bound_nsec, const int thread_relative_deadline_nsec, const bool enable_realtime_scheduling, bool* kill_this_node)
{
    realtime_scheduling_enabled_ = enable_realtime_scheduling;

    kill_this_node_ = kill_this_node;

    node_prefix_ = ros::this_node::getName();

    subscriber_node_handle_.setCallbackQueue(&subscriber_callback_queue_);
    subscriber_target_joint_positions_ = subscriber_node_handle_.subscribe(node_prefix_+"/set/target_joint_positions", 1, &DensoCommunicationDummyNode::updateTargetJointPositionsCallback, this);

    publisher_node_handle_.setCallbackQueue(&publisher_callback_queue_);
    publisher_joint_state_ = publisher_node_handle_.advertise<sensor_msgs::JointState>(node_prefix_+"/get/joint_state",1);

    datalogger_node_handle_.setCallbackQueue(&datalogger_callback_queue_);
    datalogger_ = new rosilo::DataloggerInterface(datalogger_node_handle_,4);

    //Initialize vectors
    target_joint_positions_  = VectorXd::Zero(6);
    joint_positions_         = VectorXd::Zero(6);

    //Initialize ros messages
    publisher_joint_state_msg_.effort.resize(6,0.0);
    publisher_joint_state_msg_.position.resize(6,0.0);
    publisher_joint_state_msg_.velocity.resize(6,0.0);

    thread_estimated_computation_time_upper_bound_nsec_ = thread_estimated_computation_time_upper_bound_nsec;
    thread_relative_deadline_nsec_ = thread_relative_deadline_nsec;
    thread_sampling_time_nsec_   = thread_sampling_time_nsec;
    thread_sampling_time_nsec_d_ = (double)thread_sampling_time_nsec_;
    thread_sampling_time_sec_d_  = thread_sampling_time_nsec_d_/rosilo::NSEC_TO_SEC_D;
    clock_             =  new rosilo::Clock(thread_sampling_time_nsec_);
}

DensoCommunicationDummyNode::~DensoCommunicationDummyNode()
{
    delete clock_;
    delete datalogger_;
}

int DensoCommunicationDummyNode::control_loop()
{
    clock_->init();

    while(not shouldShutdown())
    {
        //Sleep
        clock_->update_and_sleep();

        subscriber_callback_queue_.callAvailable();
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

    return 0;
}//End function


bool DensoCommunicationDummyNode::shouldShutdown()
{
    return (*kill_this_node_);
}

void DensoCommunicationDummyNode::shutdown()
{
    (*kill_this_node_) = true;
}


}
