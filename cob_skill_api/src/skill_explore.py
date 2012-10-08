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
# Explore Machine implemented using the skills API
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

import skill_sm_explore
from abc_skill import SkillsBase

class SkillImplementation(SkillsBase):
    
    def __init__(self):
        self.machine = self.create_machine()
        
        with self.machine:
            self.machine.add('Explore_SKILL',skill_sm_explore.skill_sm_explore(),
                     transitions={'success':'Explore_SKILL'})
        
    ####################################################################
    # function: create_machine()
    # Creates the Machine
    ####################################################################
    def create_machine(self, outcomes=['success', 'failed']):
    
        return smach.StateMachine(outcomes)
    
    def pre_conditions(self):
    
        print "Some preconditions"
    
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
    rospy.init_node('Explore')
    sm = SkillImplementation()
    sm = sm.machine
    
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
