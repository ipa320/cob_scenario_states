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
# Skill state put objects on tray re-implemented using the skills API
# *****DEVELOPER OF THE STATE****: Not specified on the repository
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
import tf
from simple_script_server import *  # import script
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *

from cob_arm_navigation_python.MoveHand import *
from cob_arm_navigation_python.MoveArm import *
from cob_arm_navigation_python.MotionPlan import *

from pr2_python import transform_listener
from pr2_python import world_interface
from pr2_python import hand_description
from pr2_python import conversions

from copy import deepcopy

from abc_state_skill import SkillsState
## Put object on tray side state
#
# This state puts a side grasped object on the tray
class skill_state_putobjectontray(SkillsState):
    
    def __init__(self, components = []):
        
        self.components = components
        self.state = self.create_state()
        self.state.execute = self.execute
        
        
    def create_state(self,outcomes=['put', 'not_put', 'failed'],input_keys=['object','graspdata']):
        
        state = smach.State()
        state.__init__(outcomes, input_keys)
        
        return state
        
    
    def execute(self, userdata):
        
        if ("light" in self.components):
            sss.set_light('blue')
            
        sss.move("head","front", False)
        
        if("light" in self.components):
            sss.set_light('yellow')
        
        handle_arm = sss.move("arm","grasp-to-tray",False)
        sss.move("tray","up")
        handle_arm.wait()
        sss.move("sdh","cylopen")
        handle_arm = sss.move("arm","tray-to-folded",False)
        rospy.sleep(3)
        sss.move("sdh","cylclosed")
        handle_arm.wait()
        # end fix
        if("light" in self.components):
            sss.set_light('green')
        return 'put'