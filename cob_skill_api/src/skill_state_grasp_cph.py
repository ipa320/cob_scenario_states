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
# Skill state grasp objects Re-Implementation using skills API
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
from simple_script_server import *  # import script
sss = simple_script_server()

from cob_srvs.srv import Trigger

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *

from cob_arm_navigation_python.MoveHand import *
from cob_arm_navigation_python.MoveArm import *
from cob_arm_navigation_python.MotionPlan import *

from pr2_python import transform_listener
from pr2_python import world_interface
from pr2_python import conversions
from pr2_python import hand_description

from copy import deepcopy
from tf.transformations import *
from tf_conversions import posemath as pm
import numpy

from abc_state_skill import SkillsState

class skill_state_grasp_cph(SkillsState):
    def __init__(self, components = []):
        
        self.components = components
        self.state = self.create_state()
        self.state.execute = self.execute
        transform_listener.get_transform_listener()
    
    def create_state(self, outcomes=['grasped','not_grasped','failed'],
            input_keys=['objects'],output_keys=['graspdata']):
        
        state = smach.State()
        state.__init__(outcomes, input_keys, output_keys)
        return state
    
    def execute(self, userdata):
        
        if ("light" in self.components):
            sss.set_light('blue')
        
        userdata.objects[0].pose.header.stamp = rospy.Time.now()+rospy.Duration(1)
        wi = WorldInterface()
        wi.reset_attached_objects()
        graspdata = dict()
        print userdata.objects

        # add wall
        wall_extent = [3.0,0.1,2.5]
        wall_pose = conversions.create_pose_stamped([  0, -0.99 - wall_extent[1], wall_extent[2]/2.0 ,0,0,0,1], 'base_link') # magic numbers are cool
        wi.add_collision_box(wall_pose, wall_extent, "wall")

        # add floor
        floor_extent = [3.0,3.0,0.1]
        floor_pose = conversions.create_pose_stamped([  0, 0, floor_extent[2]/2.0 ,0,0,0,1], '/base_link') # magic numbers are cool
        wi.add_collision_box(floor_pose, floor_extent, "floor")
        
        #transform into base_link
        grasp_pose = transform_listener.transform_pose_stamped('base_link', userdata.objects[0].pose, use_most_recent=False)

        # add object bounding box
        obj_pose = deepcopy(grasp_pose)
        lwh = userdata.objects[0].bounding_box_lwh
        m1 = pm.toMatrix( pm.fromMsg(obj_pose.pose) )
        m2 = pm.toMatrix( pm.fromTf( ((0,0, lwh.z/2.0),(0,0,0,1)) ) )
        obj_pose.pose = pm.toMsg( pm.fromMatrix(numpy.dot(m1,m2)) )
        wi.add_collision_box(obj_pose,(lwh.x*2.0,lwh.y*2.0,lwh.z) , "grasp_object")

        print grasp_pose
        # add table
        table_extent = (2.0, 2.0, grasp_pose.pose.position.z)
        table_pose = conversions.create_pose_stamped([ -0.5 - table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1], 'base_link')
        wi.add_collision_box(table_pose, table_extent, "table")

        # calculate grasp and lift pose
        grasp_pose.pose.position.x += 0.02
        grasp_pose.pose.position.y += 0.02
        grasp_pose.pose.position.z += 0.1 #0.03 + 0.05
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(-1.706, 0.113, 2.278)) # orientation of sdh_grasp_link in base_link for 'grasp' joint goal
        
        graspdata['height'] =  grasp_pose.pose.position.z - table_extent[2]
        
        pregrasp_pose = deepcopy(grasp_pose)
        pregrasp_pose.pose.position.x += 0.20
        pregrasp_pose.pose.position.y += 0.11
        pregrasp_pose.pose.position.z += 0.1

        lift_pose = deepcopy(grasp_pose)
        lift_pose.pose.position.z += 0.03
        
        mp = MotionPlan()
        # open hand
        mp += CallFunction(sss.move, 'sdh','cylopen', False)
        mp += MoveArm('arm',[pregrasp_pose,['sdh_grasp_link']], seed = 'pregrasp')
        mp += MoveComponent('sdh','cylopen')

        # allow collison hand/object
        #for l in hand_description.HandDescription('arm').touch_links:
        #    mp += EnableCollision("grasp_object", l)
        #
        # disable collison
        #mp += ResetCollisions()

        # goto grasp
        mp += MoveArmUnplanned('arm', [grasp_pose,['sdh_grasp_link']])
        
                # close hand
        mp += MoveComponent('sdh','cylclosed')
        
        # check grasp
        #mp += CheckService('/sdh_controller/is_cylindric_grasped', Trigger, lambda res: res.success.data)
        
        # attach object
        mp += AttachObject('arm', "grasp_object")
        
        # enable collision
        mp += EnableCollision("grasp_object", "table")
        
        # lift motion
        mp += MoveArmUnplanned('arm', [lift_pose,['sdh_grasp_link']])

        # disable collison
        mp += ResetCollisions()

        # move away
        mp += MoveArm('arm', [pregrasp_pose,['sdh_grasp_link']])
        
        # goto hold
        mp += MoveArm('arm', 'hold')
        
        userdata.graspdata = graspdata

        if not mp.plan(5).success:
                        if ("light" in self.components):
                            sss.set_light('green')
                        return "not_grasped"
        
        if ("light" in self.components):
            sss.set_light('yellow')
        
        if("sound" in self.components):
            sss.say(["I am grasping " + userdata.objects[0].label + " now."],False)
        # run, handle errors
        i = 0
        for ex in mp.execute():
            if not ex.wait(80.0).success:
                                if ("light" in self.components):
                                    sss.set_light('red')
                                return 'failed'
            i+=1
        if ("light" in self.components):
            sss.set_light('green')
        return 'grasped'
