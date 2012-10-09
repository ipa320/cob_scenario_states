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
# Explore State Machine using the Skills API
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
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import random
from nav_msgs.msg import Odometry

from simple_script_server import *
sss = simple_script_server()

from abc_sm_skill import SkillsSM

import skill_approachpose
import skill_detectobjectsfront
import skill_detectobjectsback
import skill_state_announcefoundobjects
import skill_grasp
import skill_selectnavgoal


class skill_sm_explore(SkillsSM):

    def __init__(self):

        self.machine = self.create_machine()
        
        self.nav_goal = skill_selectnavgoal.SkillImplementation()
        self.approach_pose = skill_approachpose.SkillImplementation()
        self.detect_front = skill_detectobjectsfront.SkillImplementation()
        self.announce_front = skill_state_announcefoundobjects.skill_state_announcefoundobjects()
        self.detect_back = skill_detectobjectsback.SkillImplementation()
        self.announce_back = skill_state_announcefoundobjects.skill_state_announcefoundobjects()
        self.grasp = skill_grasp.SkillImplementation()

        with self.machine:
            
            self.machine.add('SELECT_NAVIGATION_GOAL', self.nav_goal.machine, transitions={'selected':'APPROACH_POSE_SKILL','not_selected':'failed','failed':'failed'})

            self.machine.add('APPROACH_POSE_SKILL',self.approach_pose.machine,
                     transitions={'reached':'DETECT_FRONT_SKILL',
                                  'not_reached': 'SELECT_NAVIGATION_GOAL',
                                  'failed_pre_condition_check': 'failed',
                                  'failed_post_condition_check': 'failed',
                                  'failed':'failed'})

            self.machine.add('DETECT_FRONT_SKILL',self.detect_front.machine,
                     transitions={'detected':'ANNOUNCE_FRONT_SKILL',
                                  'not_detected':'ANNOUNCE_FRONT_SKILL',
                                  'failed':'failed'})

            self.machine.add('ANNOUNCE_FRONT_SKILL',self.announce_front.state,
                     transitions={'announced':'GRASP_SKILL',
                                  'not_announced':'DETECT_BACK_SKILL',
                                  'failed':'failed'})

            self.machine.add('DETECT_BACK_SKILL', self.detect_back.machine,
                     transitions={'detected':'ANNOUNCE_BACK_SKILL',
                                  'not_detected':'ANNOUNCE_BACK_SKILL',
                                  'failed':'failed'})

            self.machine.add('ANNOUNCE_BACK_SKILL',self.announce_back.state,
                     transitions={'announced':'GRASP_SKILL',
                                  'not_announced':'SELECT_NAVIGATION_GOAL',
                                  'failed':'failed'})

            self.machine.add('GRASP_SKILL',self.grasp.machine,
                     transitions={'grasped':'SELECT_NAVIGATION_GOAL',
                                  'not_grasped':'SELECT_NAVIGATION_GOAL',
                                  'failed':'failed'})

    ####################################################################
    # function: create_machine()
    # Creates the Machine
    ####################################################################
    
    def create_machine(self, outcomes=['success', 'failed']):
    
        return smach.StateMachine(outcomes)