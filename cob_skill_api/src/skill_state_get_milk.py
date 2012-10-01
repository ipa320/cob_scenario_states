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
# Brings the Get Milk as an example for the Skills API
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

import time

import roslib; roslib.load_manifest('cob_script_server')
import rospy

import smach
import smach_ros

from simple_script_server import script

import tf
from geometry_msgs.msg import *

from abc_state_skill import SkillsState

from simple_script_server import *
sss = simple_script_server()

class skill_state_get_milk(SkillsState):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
    
        listener = tf.TransformListener(True, rospy.Duration(10.0))
        
        # move arm to pregrasp position
        handle_arm = sss.move("arm", "pregrasp", False)
        handle_sdh = sss.move("sdh", "cylopen", False)
        
        # wait for arm movements to finish
        handle_arm.wait()
        handle_sdh.wait()
        
        # calculate tranformations, cup coordinates needed for coordination
        cup = PointStamped()
        cup.header.stamp = rospy.Time.now()
        cup.header.frame_id = "/map"
        cup.point.x = -0.3
        cup.point.y = -1.39
        cup.point.z = 0.8
        sss.sleep(2)
        
        if not sss.parse:
            cup = listener.transformPoint('/arm_7_link', cup)
        
        # grasp milk box
        sss.move("arm",'pregrasp_2')
        sss.move("arm", "grasp")#sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
        sss.move("sdh", "cylclosed")
        
        # move milk box
        #sss.move_cart_rel("arm",[[0.0, 0.4, 0.0], [0, 0, 0]])
        
        return 'success'

class INIT(smach.State):

    def __init__(self):
    
        smach.State.__init__(self, outcomes=['goto'])

    def execute(self, userdata):
    
        # initialize components (not needed for simulation)
        sss.init("torso")
        sss.init("tray")
        sss.init("arm")
        sss.init("sdh")
        sss.init("head")
        
        sss.init("base")
        
        return 'goto'


class INIT_POS(smach.State):

    def __init__(self):
    
        smach.State.__init__(self, outcomes=['goto'])

    def execute(self, userdata):
    
        # move to initial positions
        handle_torso = sss.move("torso", "home", False)
        handle_tray = sss.move("tray", "down", False)
        handle_arm = sss.move("arm", "folded", False)
        handle_sdh = sss.move("sdh", "cylclosed", False)
        handle_head = sss.move("head", "back", False)
        
        # wait for initial movements to finish
        handle_torso.wait()
        handle_tray.wait()
        handle_arm.wait()
        handle_sdh.wait()
        handle_head.wait()
        
        # move base to initial position
        handle_base = sss.move("base", "table", False)
        
        # wait for base to reach initial position
        handle_base.wait()
        
        return 'goto'
    
def main():

    rospy.init_node('smach_script_GET_MILK')
    
    # Create a SMACH state machine
    GET_MILK = smach.StateMachine(outcomes=['success'])
    
    # Open the container
    with GET_MILK:
    
        # Add states to the container
        smach.StateMachine.add('Initiate', INIT(), transitions={'goto':'Initial_Positions'})
        smach.StateMachine.add('Initial_Positions', INIT_POS(), transitions={'goto':'Grasp_Milk'})
        smash.StateMachine.add('Grasp_Milk', GRASP(), transitions={'goto':'success'})
    
    sis = smach_ros.IntrospectionServer('GET_MILK', GET_MILK, 'GET_MILK')
    sis.start()
    
    # Execute SMACH plan
    outcome = GET_MILK.execute()
    
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()