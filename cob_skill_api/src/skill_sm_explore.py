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

class skill_sm_explore(SkillsSM):

	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['success', 'failed'])

		self.apose = None
		
		with self:

			self.add('APPROACH_SKILL',self.mach_approach(),
                transitions={'success':'DETECT_SKILL_FRONT',
                    'failed':'APPROACH_SKILL'})

			self.add('DETECT_SKILL_FRONT',self.mach_detect(),
                transitions={'ended':'ANNOUNCE_SKILL_FRONT'})

			self.add('ANNOUNCE_SKILL_FRONT',skill_state_announcefoundobjects.skill_state_announcefoundobjects(),
                transitions={'announced':'DETECT_SKILL_BACK',
                    'not_announced':'DETECT_SKILL_BACK',
                        'failed':'failed'})

			self.add('DETECT_SKILL_BACK',self.mach_detect_back(),
                transitions={'ended':'ANNOUNCE_SKILL_BACK'})

			self.add('ANNOUNCE_SKILL_BACK',skill_state_announcefoundobjects.skill_state_announcefoundobjects(),
                transitions={'announced':'APPROACH_SKILL',
                    'not_announced':'APPROACH_SKILL',
                        'failed':'failed'})

	def mach_approach(self):
		rospy.loginfo("Executing the Approach pose Skill!")
		self.apose =  skill_approachpose.SkillImplementation()
		return self.apose

	def mach_detect(self):
		rospy.loginfo("Executing the Detect Skill!")
		mach =  skill_detectobjectsfront.SkillImplementation(components = self.apose.check_pre.full_components)
		return mach
	
	def mach_detect_back(self):
		rospy.loginfo("Executing the Detect Skill!")
		mach =  skill_detectobjectsback.SkillImplementation(components = self.apose.check_pre.full_components)
		return mach