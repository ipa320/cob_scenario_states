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
# Definition of the conditions checks for the Skill API
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
from actionlib import *
from actionlib.msg import *
from diagnostic_msgs.msg import DiagnosticArray

import tf
from tf.msg import tfMessage 
from tf.transformations import euler_from_quaternion

from simple_script_server import *  # import script
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *
from move_base_msgs.msg import *

from abc_conditioncheck import ConditionCheck

import types

class ConditionCheck(ConditionCheck):

	def __init__(self, checkType, tfL):
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['base_pose'])

		self.checkType = checkType
		self.checks = rospy.get_param(self.checkType)
		self.robot_name = rospy.get_param('robot_name')
		
		if rospy.has_param('required_components') and rospy.has_param('optional_components'):
			self.required_components = rospy.get_param('required_components')
			self.optional_components = rospy.get_param('optional_components')
			self.full_components = self.required_components + " " + self.optional_components

		elif rospy.has_param('required_components'):
			self.required_components = rospy.get_param('required_components')
			self.optional_components = ""
			self.full_components = self.required_components

		elif rospy.has_param('optional_components'):
			self.optional_components = rospy.get_param('optional_components')
			self.required_components = ""
			self.full_components = self.optional_components

		else:
			self.required_components = ""
			self.optional_components =""
			self.full_components = ""

		self.required_components = self.required_components.split()
		self.optional_components = self.optional_components.split()
		self.full_components = self.full_components.split()

		self.tfL = tfL

		self.result = "failed"

		self.status = 0

		rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnostics_callback)

	####################################################################
	# function: execute()
	# Main routine of the State Machine
	####################################################################

	def execute(self, userdata):	

		for check in self.checks:

				getattr(self, check.keys()[0])(check, userdata)

				if (self.result == "failed"):
					rospy.logerr("Check failure on <<%s>>"%check)
					return self.result

		# announce ready
		if ("sound" in self.full_components) and (self.result == "success"):
			sss.say(["Ready."])

		return self.result

	#####################################################################
	# function: joint_configuration_check()
	# This function is responsible for checking the 
	# Joint configuration of the Robot before performing the skill action
	#####################################################################

	def joint_configuration_check_js(self, params, userdata): # from parameter server, get names and supposed states

		rospy.loginfo("<<joint_configuration_check_js>>")
		rospy.loginfo("This is part of the <<%s>>", self.checkType)

		for item in params.values()[0]:	

			joint_names = item['joint_names']
			joint_states = item['joint_states']
			aw_error = item['allowed_error']

			self.joint_configuration_check(joint_names,joint_states, aw_error)


	def joint_configuration_check_ss(self, params, userdata):
		
		rospy.loginfo("<<joint_configuration_check_ss>>")

		for item in params.values()[0]:

			component = item['component']
			configuration = item['configuration']

			ss_names_path = "/script_server/" + component + "/joint_names"
			ss_values_path = "/script_server/" + component + "/" + configuration

			joint_names = rospy.get_param(ss_names_path)
			joint_states = rospy.get_param(ss_values_path)[0]
		
			aw_error = item['allowed_error']

			self.joint_configuration_check(joint_names,joint_states, aw_error)

	def joint_configuration_check(self, joint_names, joint_states, aw_error):

		joints = zip(joint_names, joint_states)

		try:
			
			
			for name, state in joints:

				rospy.loginfo("Checking the <<%s>> joint"%name)
				
				jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

				while name not in jointsMsg.name:
					jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

				value = jointsMsg.position[jointsMsg.name.index(name)]

				assert abs(value - state) <= aw_error, "Error on the Joint Position for the <<%s>>"%name
			
		except AssertionError,e:
			self.result = "failed"
			rospy.logerr("<<Error Message>>:%s"%e)
			rospy.logerr("at joint_configuration_check")
			return


		self.result = "success"

	####################################################################
	# function: component_ready_check()
	# This function is responsible for checking the initial conditions
	# provided by the user for proper operation of the Robot
	#####################################################################

	def component_ready_check(self, params, userdata): #Simplified due to the current simulation possibilities

		try:
			rospy.loginfo("Started to check if all components are ready")		

			assert self.status == 0, "<<base>> component is not ready yet."

			rospy.loginfo("All Necessary components are present.")

			self.init_components()

			rospy.loginfo("Components successfully initialized.")

		except AssertionError,e:
			self.result = "failed"
			rospy.logerr("<<Error Message>>:%s"%e)
			rospy.logerr("at:%s"%params)
			return

		self.result = "success"


	####################################################################
	# function: action_check()
	# This function is responsible for checking the availability of an 
	# action
	#####################################################################


	def action_check(self, params, userdata):

		rospy.loginfo("Checking <<actions>>")

		for item in params.values()[0]:

			action_type = item['action_type']
			action_name = item['action_name']

			rospy.loginfo("Now checking <<%s>>", action_name)
			rospy.loginfo("Of type <<%s>>", action_type)

			mod = __import__(action_name, fromlist=[action_type])
			cls = getattr(mod, action_type)

			ac_client = actionlib.SimpleActionClient(action_name, cls)

			ac_client.wait_for_server(rospy.Duration(5))	

		rospy.loginfo("Finished Checking <<actions>>")

	
	####################################################################
	# function: pose_check()
	# This function is responsible for checking if the Robot position
	# is convenient for performing the skill action
	####################################################################

	def pose_check(self, params, userdata):
		
		try:
			rospy.loginfo("Checking the Robot <<Pose>>")
			rospy.loginfo("This is part of the <<%s>>", self.checkType)

			for item in params.values()[0]:

				reference_frame = item['reference_frame']
				target_frame = item['target_pose']['frame_id']
				
				mes = "Checking the " + reference_frame + " against the " + target_frame

				rospy.loginfo(mes)

				self.tfL.waitForTransform(target_frame, reference_frame, rospy.Time(), rospy.Duration(20.0))

				(trans,rot) = self.tfL.lookupTransform(target_frame, reference_frame, rospy.Time(0))

				angles = euler_from_quaternion(rot)
		
				xy_goal_tolerance = item['allowed_position_error']
				yaw_goal_tolerance = item['allowed_orientation_error']

				if (item['target_pose']['position'] == "userdata"): # kept this for analyzing user defined goals(post_check)

					assert abs(trans[0] - userdata.base_pose[0]) <= xy_goal_tolerance, "Error on the X axis position"
					assert abs(trans[1] - userdata.base_pose[1]) <= xy_goal_tolerance, "Error on the Y axis position"
					assert abs(angles[2] - userdata.base_pose[2]) <= yaw_goal_tolerance, "Error on the Angle"

				else:
					
					for pos in range(len(trans)):
						
					
						messageX = (str)(trans[pos]) + " " + (str)(item['target_pose']['position'][pos]) + " "+ (str)(xy_goal_tolerance)
						assert abs(trans[pos] - item['target_pose']['position'][pos]) <= xy_goal_tolerance, "Error on the position %s"%messageX

						rospy.loginfo(messageX)

					for ori in range(len(angles)):
						
						messageA = (str)(angles[ori]) + " " + (str)(item['target_pose']['orientation'][ori]) + " "+ (str)(yaw_goal_tolerance)
						assert abs(angles[ori] - item['target_pose']['orientation'][ori]) <= yaw_goal_tolerance, "Error on the orientation %s"%messageA

						rospy.loginfo(messageA)
		
		except AssertionError,e:
			self.result = "failed"
			rospy.logerr("<<Error Message>>:%s"%e)		
			rospy.logerr("at pose_check")
			return
		
		
		self.result = "success" 
	

	####################################################################
	# function: init_components()
	# Initializes robot components according to the components available
	# for the Robot
	####################################################################

	def init_components(self):

		sss.sleep(2)

		if "sound" in self.full_components:
			sss.say(["Preparing."],False)
		# bring robot into the starting state

		if "tray" in self.full_components:
			handle_tray = sss.move("tray","down",False)
			handle_tray.wait()

		if "torso" in self.full_components:
			handle_torso = sss.move("torso","home",False)
			handle_torso.wait()

		if "arm" in self.full_components:
			handle_arm = sss.move("arm","folded",False)
			handle_arm.wait()

		if "arm_left" in self.full_components:
			handle_arm = sss.move("arm_left","folded",False)
			handle_arm.wait()

		if "arm_right" in self.full_components:
			handle_arm = sss.move("arm_right","folded",False)
			handle_arm.wait()


		if "sdh" in self.full_components:
			handle_sdh = sss.move("sdh","cylclosed",False)
			handle_sdh.wait()

		if "head" in self.full_components:
			handle_head = sss.move("head","front",False)
			handle_head.wait()

		# wait for all movements to be finished
		# announce ready

		if "sound" in self.full_components:
			sss.say(["Ready."])

		sss.sleep(2)

	def diagnostics_callback(self, msg):

		self.status = msg.status[0].level
