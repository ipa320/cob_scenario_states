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
# Skill for selecting Navigation Goals
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

from abc_skill import SkillsBase
import skill_state_selectnavgoal

import random

class SkillImplementation(SkillsBase):

    def __init__(self, defined_goal=None, random_conditions=[0.0, 4.0, 2.0, -4.0, 0.0, 2.0, -3.14, 3.14, 2*3.1414926/4]):
        
        rospy.loginfo("Executing the select navigation goal")
        
        
        self.defined_goal = defined_goal
        self.random_conditions = random_conditions
        
        if (self.defined_goal == None):
            self.nav_goal = skill_state_selectnavgoal.skill_state_selectrandomnav(conditions=self.random_conditions)
        else:
            self.nav_goal = skill_state_selectnavgoal.skill_state_definednav(positions=self.defined_goal)
        
        self.machine = self.create_machine() 
        
        with self.machine:
                
            self.machine.add('SELECT_NAV_GOAL', self.nav_goal.state ,transitions={'selected':'selected','not_selected':'not_selected','failed':'failed'})

       ####################################################################
    # function: create_machine()
    # Creates the Machine
    ####################################################################
    def create_machine(self, outcomes=['selected','not_selected','failed'], output_keys=['pose']):
    
        return smach.StateMachine(outcomes, output_keys)
    
    def pre_conditions(self):

        print "Some pre conditions"

    def post_conditions(self):
        print "Some post conditions"

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

    rospy.init_node('skill_select_nav')

# for pre-defined navigation Goals
    sm = SkillImplementation()
    sm = sm.machine
    
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()