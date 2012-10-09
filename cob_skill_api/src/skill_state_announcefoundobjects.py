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
# Skill state announce found objects using the skills API
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

from simple_script_server import *
sss = simple_script_server()

from abc_state_skill import SkillsState

class skill_state_announcefoundobjects(SkillsState):
    
    def __init__(self):
        rospy.loginfo("Initializing Announce objects skill")
        
        self.state = self.create_state()
        self.state.execute = self.execute
        
    ####################################################################
    # function: create_state()
    # Creates the State
    ####################################################################
    
    def create_state(self, outcomes=['announced','not_announced','failed'],
                             input_keys=['objects'],
                             output_keys=['objects']):
    
        state = smach.State()
        state.__init__(outcomes, input_keys, output_keys) 
        return state
    
    def execute(self, userdata):
        rospy.loginfo("Executing Announce Objects Skill")
        object_names = ""
        for obj in userdata.objects:
            object_names += obj.label + ", "
        
        if object_names != "":
            sss.say(["I found: " + object_names])
        else:
            sss.say(["I found: nothing"])
            return "not_announced"
        #userdata.objects = []
        
        return 'announced'