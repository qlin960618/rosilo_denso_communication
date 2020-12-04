"""
# Copyright (c) 2012-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_denso_communication.
#
#    rosilo_denso_communication is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_denso_communication is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_denso_communication.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
"""

# System
from dqrobotics import *

# ROS
import roslib

roslib.load_manifest('rosilo_denso_communication')
import rospy

# ROS Messages
from sensor_msgs.msg import JointState as rosmsg_JointState
from std_msgs.msg import Float64MultiArray as rosmsg_Float64MultiArray

class DensoCommunicationInterface:

    def __init__(self, node_prefix):

        self.enabled_ = False
        self.joint_positions_ = None

        self.publisher_target_joint_positions_ = rospy.Publisher(node_prefix+"set/target_joint_positions",
                                                                  rosmsg_Float64MultiArray,
                                                                  queue_size=1)

        self.subscriber_joint_positions_ = rospy.Subscriber(node_prefix+"get/joint_state", rosmsg_JointState, self._get_joint_state_callback)

    def set_target_joint_positions(self, target_positions):
        msg = rosmsg_Float64MultiArray(data=target_positions)
        self.publisher_target_joint_positions_.publish(msg)

    def get_joint_positions(self):
        if self.joint_positions_ is None:
            raise Exception("Tried to obtain uninitialized get_joint_positions()")
        return self.joint_positions_

    def is_enabled(self):
        return self.enabled_

    def _get_joint_state_callback(self, msg):
        self.joint_positions_ = msg.position
        if not self.enabled_:
            self.enabled_ = True
