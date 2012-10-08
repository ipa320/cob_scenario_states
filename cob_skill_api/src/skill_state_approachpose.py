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
# \date Date of creation: August 2012
#
# \brief
# Skill state implementation using the skills API
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
import random
from nav_msgs.msg import Odometry

from simple_script_server import *
sss = simple_script_server()

from abc_state_skill import SkillsState

class skill_state_approachpose(SkillsState):
    
    def __init__(self, components = [], pose = "", mode = "omni", timeout = 30.0):
        
        self.state = self.create_state()
        self.state.execute = self.execute
    
    # Subscriber to base_odometry
        rospy.Subscriber("/base_controller/odometry", Odometry, self.callback)
    
        self.components = components
    
        self.pose = pose
        self.mode = mode
        self.is_moving = False
        self.warnings = ["I can not reach my target position because my path or target is blocked.","My path is blocked.", "I can not reach my target position."]
        self.timeout = timeout
    
    
    def execute(self, userdata):
        
        # determine target position
        if self.pose != "":
            pose = self.pose
        elif type(userdata.pose) is str:
            pose = userdata.pose
        elif type(userdata.pose) is list:
            pose = []
            pose.append(userdata.pose[0])
            pose.append(userdata.pose[1])
            pose.append(userdata.pose[2])
        else: # this should never happen
            rospy.logerr("Invalid userdata 'pose'")
            return 'failed'

        # try reaching pose
        handle_base = sss.move("base", pose, mode=self.mode, blocking=False)

        # init variables
        stopping_time = 0.0
        announce_time = 0.0
        freq = 2.0 # Hz
        yellow = False

        # check for goal status
        while not rospy.is_shutdown():

            # finished with succeeded
            if (handle_base.get_state() == 3):
                if ("light" in self.components):
                    sss.set_light('green')
                return 'reached'
            # finished with aborted
            elif (handle_base.get_state() == 4):
                if ("light" in self.components):
                    sss.set_light('green')
                return 'not_reached'
            # finished with preempted or canceled
            elif (handle_base.get_state() == 2) or (handle_base.get_state() == 8):
                if ("light" in self.components):
                    sss.set_light('green')
                return 'not_reached'
            # return with error
            elif (handle_base.get_error_code() > 0):
                print "error_code = " + str(handle_base.get_error_code())
                if ("light" in self.components):
                    sss.set_light('red')
                return 'failed'

            # check if the base is moving
            loop_rate = rospy.Rate(freq) # hz
            if not self.is_moving: # robot stands still            
                # increase timers
                stopping_time += 1.0/freq
                announce_time += 1.0/freq

                # abort after timeout is reached
                if stopping_time >= self.timeout:
                    sss.stop("base")
                    if ("light" in self.components):
                        sss.set_light('green')
                    return 'not_reached'

                # announce warning after every 10 sec
                if announce_time >= 10.0:
                    if "sound" in self.components:
                        sss.say([self.warnings[random.randint(0,len(self.warnings)-1)]],False)
                    announce_time = 0.0

                # set light to "thinking" after not moving for 2 sec
                if round(stopping_time) >= 2.0:
                    if ("light" in self.components):
                        sss.set_light("blue")
                    yellow = False
            else:
                # robot is moving
                if not yellow:
                    if ("light" in self.components):
                        sss.set_light("yellow")
                    yellow = True

            # sleep
            loop_rate.sleep()

    ####################################################################
    # function: create_state()
    # Creates the State
    ####################################################################
    def create_state(self, outcomes=['reached', 'not_reached', 'failed'],
                             input_keys=['pose'],output_keys=[]):
    
        return smach.State(outcomes, input_keys, output_keys)
    
    #Callback for the /base_controller/odometry subscriber
    def callback(self,msg):
        r = 0.01 # error range in m/s or rad/s
        if (abs(msg.twist.twist.linear.x) > r) or (abs(msg.twist.twist.linear.y) > r) or (abs(msg.twist.twist.angular.z) > r):
            self.is_moving = True
        else:
            self.is_moving = False
        return
