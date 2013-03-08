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
# Skill state grasp objects Re-Implementation using skills API
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
import geometry_msgs
import random

import condition_check
#import skill_state_grasp
import skill_state_grasp_cph
import skill_state_putobjectontray

import tf
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion


from cob_object_detection_msgs.msg import * # for defining the main
from cob_object_detection_msgs.srv import *  # for defining the main


from simple_script_server import *
sss = simple_script_server()

class SkillImplementation(SkillsBase):

    def __init__(self):
        
        rospy.loginfo("Executing the grasp machine")
        
        self.machine = self.create_machine()
        
        self.check_pre = self.pre_conditions()
        self.grasp = self.execute_machine()
        self.put_on_tray = skill_state_putobjectontray.skill_state_putobjectontray(components = self.check_pre.full_components)
        
        with self.machine:
            
            self.machine.add("PRECONDITIONS_GRASP", self.check_pre.state, transitions={'success' : "GRASP_CPH",'failed':'failed'})
            
            self.machine.add('GRASP_CPH',self.grasp.state,
                                transitions={'not_grasped':'failed',
                                        'failed':'failed','grasped':'PUT_ON_TRAY'})
            
            self.machine.add("PUT_ON_TRAY", self.put_on_tray.state, 
                             transitions={'put':"success",
                                          "not_put":"failed", 'failed':'failed'})
      ####################################################################
    # function: create_machine()
    # Creates the Machine
    ####################################################################
    def create_machine(self, outcomes=['grasped','not_grasped', 'failed', 'success'],
            input_keys=['objects']):
    
        return smach.StateMachine(outcomes,input_keys)
    
    def execute_machine(self):
        rospy.loginfo("Executing the Grasp Skill!")
        mach =  skill_state_grasp_cph.skill_state_grasp_cph(components = self.check_pre.full_components)
        return mach
    
    def pre_conditions(self):

        self.check_pre = condition_check.ConditionCheck(checkType="pre_grasp_check")
        return self.check_pre

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
    
        rospy.init_node('grasp_object')
        
        sm = SkillImplementation()
        sm = sm.machine
        
        sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
        sis.start()
        
        # Creating the userdata for performing the grasp    
        obj1 = cob_object_detection_msgs.msg.Detection()
        obj1.header.frame_id = ""
        obj1.header.stamp = rospy.Time.now()
        obj1.label = "milk"
        obj1.detector = "Testingthegrasp"

        obj1.pose.pose.position.x = 0.0074015454892
        obj1.pose.pose.position.y = -0.319340167725
        obj1.pose.pose.position.z = 1.03468273568

        obj1.pose.pose.orientation.x = -0.0330447723319
        obj1.pose.pose.orientation.y = 0.730206066455
        obj1.pose.pose.orientation.z = 0.682365125033
        obj1.pose.pose.orientation.w = 0.00921844197269
        
        obj1.pose.header.frame_id = "/head_color_camera_l_link"
        obj1.pose.header.stamp = rospy.Time.now()+rospy.Duration(1)
        
        sm.userdata.objects = [obj1]
        
        outcome = sm.execute()
        rospy.spin()