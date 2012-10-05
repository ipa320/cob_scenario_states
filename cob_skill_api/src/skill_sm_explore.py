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

        smach.StateMachine.__init__(self,
                                    outcomes=['success', 'failed'])

        with self:
            
            self.add('SELECT_NAVIGATION_GOAL',skill_selectnavgoal.SkillImplementation(), transitions={'selected':'APPROACH_POSE_SKILL','not_selected':'failed','failed':'failed'})

            self.add('APPROACH_POSE_SKILL',skill_approachpose.SkillImplementation(),
                     transitions={'reached':'DETECT_FRONT_SKILL',
                                  'not_reached': 'SELECT_NAVIGATION_GOAL',
                                  'failed_pre_condition_check': 'failed',
                                  'failed_post_condition_check': 'failed',
                                  'failed':'failed'})

            self.add('DETECT_FRONT_SKILL',skill_detectobjectsfront.SkillImplementation(),
                     transitions={'detected':'ANNOUNCE_FRONT_SKILL',
                                  'not_detected':'ANNOUNCE_FRONT_SKILL',
                                  'failed':'failed'})

            self.add('ANNOUNCE_FRONT_SKILL',skill_state_announcefoundobjects.skill_state_announcefoundobjects(),
                     transitions={'announced':'GRASP_SKILL',
                                  'not_announced':'DETECT_BACK_SKILL',
                                  'failed':'failed'})

            self.add('DETECT_BACK_SKILL', skill_detectobjectsback.SkillImplementation(),
                     transitions={'detected':'ANNOUNCE_BACK_SKILL',
                                  'not_detected':'ANNOUNCE_BACK_SKILL',
                                  'failed':'failed'})

            self.add('ANNOUNCE_BACK_SKILL',skill_state_announcefoundobjects.skill_state_announcefoundobjects(),
                     transitions={'announced':'GRASP_SKILL',
                                  'not_announced':'SELECT_NAVIGATION_GOAL',
                                  'failed':'failed'})

            self.add('GRASP_SKILL',skill_grasp.SkillImplementation(),
                     transitions={'grasped':'SELECT_NAVIGATION_GOAL',
                                  'not_grasped':'SELECT_NAVIGATION_GOAL',
                                  'failed':'failed'})