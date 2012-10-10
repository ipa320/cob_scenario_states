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
import skill_state_grasp

import tf
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion

import skill_state_grasp
from cob_object_detection_msgs.msg import * # for defining the main
from cob_object_detection_msgs.srv import *  # for defining the main


from simple_script_server import *
sss = simple_script_server()

class SkillImplementation(SkillsBase):

    def __init__(self):
        
        rospy.loginfo("Executing the grasp machine")
        
        self.machine = self.create_machine()
        self.grasp = skill_state_grasp.skill_state_grasp()
        self.check_pre = self.pre_conditions()
                
        with self.machine:
            
            self.machine.add("PRECONDITIONS_GRASP", self.check_pre.state, transitions={'success' : "GRASP_SIDE",'failed':'failed'})
            
            self.machine.add('GRASP_SIDE',skill_state_grasp.grasp_side(),
                                transitions={'not_grasped':'failed',
                                        'failed':'failed','grasped':'success'})
           # self.machine.add('GRASP_TOP',skill_state_grasp.grasp_top(),
           #                     transitions={'not_grasped':'failed',
           #                             'failed':'failed','grasped':'success'})
            
      ####################################################################
    # function: create_machine()
    # Creates the Machine
    ####################################################################
    def create_machine(self, outcomes=['grasped','not_grasped', 'failed', 'success'],
            input_keys=['objects']):
    
        return smach.StateMachine(outcomes,input_keys)
    
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
            
        obj1 = cob_object_detection_msgs.msg.Detection()
        obj1.header.frame_id = "/head_color_camera_l_link"
        obj1.header.stamp = rospy.Time.now()
        obj1.label = "milk"
        obj1.detector = "Testingthegrasp"
        
        #obj1.pose.pose.position = [-0.0548130202307,-0.333948243415,0.914694305204]
        obj1.pose.pose.position.x = -0.0548130202307
        obj1.pose.pose.position.y = -0.333948243415
        obj1.pose.pose.position.z = 0.914694305204

        obj1.pose.pose.orientation.x = -0.0232504826646
        obj1.pose.pose.orientation.y = 0.726799172609
        obj1.pose.pose.orientation.z = 0.686439171832
        obj1.pose.pose.orientation.w = -0.00486221397491
        obj1.pose.header.frame_id = "/head_color_camera_l_link"
        obj1.pose.header.stamp = rospy.Time.now()
        #obj1.pose.stamp = rospy.Time.now()
        sm.userdata.objects = [obj1]
        
        outcome = sm.execute()
        rospy.spin()