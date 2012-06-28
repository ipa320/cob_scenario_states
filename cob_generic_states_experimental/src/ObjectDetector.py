#!/usr/bin/python

#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_scenarios
# \note
#   ROS package name: cob_generic_states_experimental
#
# \author
#   Ulrich Reiser, email:ulrich.reiser@ipa.fhg.de
#
# \date Date of creation: June 26 2012
#
# \brief
#   Implements generic detection routine which is used in multiple detection states.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
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
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################


import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from math import *
import copy

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *


## detect_object
#
# In this class the actual object detection is executed exactly one time
# No further actions are executed
# \param namespace Indicates the namespace for the torso poses pushed onto the parameter server (we could have several detection states
#        with different desired torso poses)
# \param object_names Input list containing the objects that are looked for
# \param detector_srv Detection service that is called for the actual obhect detection
#    	 e.g. /object_detection/detect_object
# \param mode logical mode for detection result \	
#	 all: outcome is success only if all objects in object_names are detected
#	 one: outcome is success if at least one object from object_names list is detected 

class ObjectDetector:
	def __init__(self, namespace, object_names = [], detector_srv = '/object_detection/detect_object', mode = 'all'  ):
		self.detector_srv = detector_srv 
		self.object_names = object_names
		self.detected_objects = []
		self.torso_poses = []

	
		# get torso poses that should be used for the detection from Parameter Server
		if rospy.has_param(namespace):
			params = rospy.get_param(namespace)
			torso_poses = params("torso_poses")
			
			rospy.loginfo("Found %d torso poses for state %s on ROS parameter server" %len(torso_poses) %namespace, )
			for pose in torso_poses:
				if (pose[0] == 'joints'): #(joints,0;0;0)
					self.torso_poses.append((pose[1],pose[2],pose[3]))
				elif (pose[0] == 'xyz'): #(xyz;0;5;3)
					#TODO call look at point in world (MDL)
					print "Calling LookAtPointInWorld"
				else: #string:
					self.torso_poses.append(pose)
		else:
			rospy.loginfo("No torso_poses found for state %s on ROS parameter server, taking 'home' as default" %namespace)
			self.torso_poses.append("home") # default pose
		
		
		if mode not in ['all','one']:
			rospy.logwarn("Invalid mode: must be 'all', or 'one', selecting default value = 'all'")
			self.mode = 'all'	
		else:
			self.mode = mode

	def execute(self, userdata):

		# determine object name
		if self.object_names != []:
			object_names = self.object_names
		elif type(userdata.object_names) is list:
			for name in userdata.object_names:
				if type(name) is not str:
					rospy.logerr("Invalid userdata: must be list of strings")
					return 'failed'
			object_names = userdata.object_names
		else: # this should never happen
			rospy.logerr("Invalid userdata 'object_names'")
			return 'failed'

		# check if object detection service is available
		print self.detector_srv
		try:
			rospy.wait_for_service(self.detector_srv,10)
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			return 'failed'
	
		#iterate through torso poses until objects have been detected according to the mode	
		for pose in self.torso_poses:
			# have an other viewing point for each retry
			handle_torso = sss.move("torso",pose)
		
			# call object detection service
			try:
				sss.say(["I am now looking for the asked objects"],False)
				detector_service = rospy.ServiceProxy(self.detector_srv, DetectObjects)
				req = DetectObjectsRequest()
				req.object_names.data = object_names
				res = detector_service(req)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
				return 'failed'
				
			#merge detection into detected_object list
			for _object in res.object_list.detections:
				# TODO check if object with same label is inside bounding box
				self.detected_objects.append(_object)

			#check if required  objects are detected
			detected_all = true
			if mode == 'one':
				for _searched_object in object_names:
					for _object in self.detected_objects:
						if _object.label == _searched_object:
							return 'detected', self.detected_objects
			elif mode == 'all':
				for _searched_object in object_names:
					for _object in self.detected_objects:
						if _searched_object != _object.label:
							detected_all = false
							break
				if detected_all == true:
					return 'detected', self.detected_objects
			
			
		
				
					
		rospy.loginfo("No objects found")
		return 'not_detected', None
				
				
				
			

			
			
		
		
