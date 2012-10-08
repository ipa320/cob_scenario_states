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
# \date Date of creation: September 2012
#
# \brief
# Skill state detect objects re-implementation using the skills API
# Detect objects at the Back Side of the Robot
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

import tf
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion

import skill_state_detectobjectsback

from simple_script_server import *
sss = simple_script_server()

class SkillImplementation(SkillsBase):

    def __init__(self, object_names = ['milk']):
    
        rospy.loginfo("Executing the detect object backside Machine")
        
        self.machine = self.create_machine()
        
        
        rospy.set_param("detect_object_table/torso_poses",['back_extreme','back_left_extreme','back_right_extreme','back','back_left','back_right'])
        
        self.check_pre = self.pre_conditions()
        self.check_post = self.post_conditions()
        self.machine.userdata.object_names = object_names
        self.detect_back = self.execute_machine()
        
        with self.machine:
            self.machine.add("PRECONDITIONS_DETECT_BACK", self.check_pre.state, transitions={'success':'DETECT_OBJECT_BACK', 'failed':'PRECONDITIONS_DETECT_BACK'})
            self.machine.add('DETECT_OBJECT_BACK',self.detect_back.state,
                     transitions={'not_detected':'not_detected',
                                  'failed':'failed','detected':'detected'})
    
     ####################################################################
    # function: create_machine()
    # Creates the Machine
    ####################################################################
    def create_machine(self, outcomes=['detected', 'not_detected', 'failed'], output_keys=['objects']):
    
        return smach.StateMachine(outcomes, output_keys)
    
    def execute_machine(self):
        rospy.loginfo("Executing the detect objects front Skill!")
        mach =  skill_state_detectobjectsback.skill_state_detectobjectsback(object_names=self.machine.userdata.object_names, components = self.check_pre.full_components)
        return mach
    
    def pre_conditions(self):
    
        check_pre = condition_check.ConditionCheck(checkType="pre_detect_object_back_check")
        return check_pre
    
    def post_conditions(self):
        print "Some postconditions"
    
    @property
    def inputs(self):
        return "Some Input"
    
    @property
    def outputs(self):
        return "Some Output"
    
    @property
    def requirements(self):
        return "Some Requirements"

if __name__=='__main__':
    rospy.init_node('detect_object_backside')
    sm = SkillImplementation()
    sm=sm.machine
    
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()