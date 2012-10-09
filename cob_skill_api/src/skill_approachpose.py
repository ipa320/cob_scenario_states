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
# \date Date of creation: August 2012
#
# \brief
# Implementation of the Skill ApproachPose
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

#!/usr/bin/python

import abc
from abc_skill import SkillsBase
import yaml

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
import random

import condition_check
import skill_state_approachpose
import skill_selectnavgoal

import tf
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion

class SkillImplementation(SkillsBase):
    def __init__(self, defined_goal=None,):

        self.machine = self.create_machine()
        
        rospy.loginfo("Started executing the ApproachPose State Machine")
        self.defined_goal = defined_goal
        self.full_components = ""
        self.required_components = ""
        self.optional_components = ""
        self.check_pre = self.pre_conditions()
        self.check_post = self.post_conditions()
        self.approach_pose = self.execute_machine()
        
        with self.machine:

            self.machine.add('PRECONDITION_CHECK',self.check_pre.state , 
                             transitions={'success':'APPROACH_POSE', 'failed':'failed_pre_condition_check'},
                             remapping={'pose':'pose'})
            self.machine.add('APPROACH_POSE',self.approach_pose.state, 
                             transitions={'reached':'POSTCONDITION_CHECK', 'failed':'failed', 'not_reached': 'not_reached'},
                             remapping={'pose':'pose'})
            self.machine.add('POSTCONDITION_CHECK',self.check_post.state, 
                             transitions={'success':'reached', 'failed':'failed_post_condition_check'},
                             remapping={'pose':'pose'})
      ####################################################################
    # function: create_state()
    # Creates the State
    ####################################################################
    def create_machine(self, outcomes=['reached', 'not_reached', 'failed', 'failed_pre_condition_check', 'failed_post_condition_check'], output_keys = ["pose"], input_keys = ["pose"]):

        return smach.StateMachine(outcomes, input_keys, output_keys)
    
    def execute_machine(self):
        rospy.loginfo("Executing the Approach pose Skill!")
        mach =  skill_state_approachpose.skill_state_approachpose(components = self.check_pre.full_components)
        return mach

    def pre_conditions(self):

        check_pre = condition_check.ConditionCheck(checkType="pre_approach_pose_check")
        return check_pre

    def post_conditions(self):

        check_post = condition_check.ConditionCheck(checkType = "post_approach_pose_check")
        return check_post

    @property
    def inputs(self):
        return "Some Input"

    @property
    def outputs(self):
        return "Some Output"

    @property
    def requirements(self):
        return "Some Requirements"


if __name__ == "__main__":

    rospy.init_node('skill_template')

# for pre-defined navigation Goals
    sm = SkillImplementation()
    sm = sm.machine
    
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    sm.userdata.pose = [4.3,-4.3,0.0]#[1,-3,0]
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
