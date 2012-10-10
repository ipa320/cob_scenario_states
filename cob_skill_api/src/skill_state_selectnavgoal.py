#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: cob_skill_api
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: October 2012
#
# \brief
# Skill state for the select navigation Goal
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
import random
from nav_msgs.msg import Odometry

from simple_script_server import *
sss = simple_script_server()

from abc_state_skill import SkillsState

class skill_state_selectrandomnav(SkillsState):
#    Needs x_min, x_max, x_increment, y_min, y_max, y_increment, th_min, th_max, th_increment
#    that provides the characteristics of the scenario for creating a random goal

    def __init__(self, conditions=[0,0,0,0,0,0,0,0,0]):
        
            self.state = self.create_state(outcomes=['selected','not_selected','failed'],output_keys=['pose'], input_keys=["pose"])
            
            self.state.execute = self.execute
            
            self.conditions = conditions
            self.goals = []
            

    def execute(self, userdata):
            

            rospy.loginfo("executing random goal selection")
            x_min, x_max, x_increment, y_min, y_max, y_increment, th_min, th_max, th_increment = self.conditions

            if len(self.goals) == 0:
                    x = x_min
                    y = y_min
                    th = th_min
                    while x <= x_max:
                            while y <= y_max:
                                    while th <= th_max:
                                            pose = []
                                            pose.append(x) # x
                                            pose.append(y) # y
                                            pose.append(th) # th
                                            self.goals.append(pose)
                                            th += th_increment
                                    y += y_increment
                                    th = th_min
                            x += x_increment
                            y = y_min
                            th = th_min

            
            userdata.pose = self.goals.pop(random.randint(0,len(self.goals)-1))
            print userdata.pose

            return 'selected'
      ####################################################################
    # function: create_state()
    # Creates the State
    ####################################################################
    def create_state(self, outcomes, output_keys, input_keys):
    
            state = smach.State()
            state.__init__(outcomes, input_keys, output_keys)
            return state
    
class skill_state_definednav(SkillsState):
    #    Needs x, y, theta
    
    def __init__(self, positions=[0,0,0]):
        
            self.state = self.create_state(outcomes=['selected','not_selected','failed'],output_keys=['pose'], input_keys=["pose"])
            
            self.state.execute = self.execute
            self.positions = positions

    def execute(self, userdata):
            rospy.loginfo("Executing defined goal selection.")
            userdata.pose = self.positions

            return 'selected'
        
    ####################################################################
    # function: create_state()
    # Creates the State
    ####################################################################
    def create_state(self, outcomes, output_keys, input_keys):

        state = smach.State()
        state.__init__(outcomes, input_keys, output_keys)
        return state
    