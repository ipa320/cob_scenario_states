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
#   ROS package name: cob_generic_states
#
# \author
#   Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2011
#
# \brief
#   Implements generic states which can be used in multiple scenarios.
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

import rospy
import os
import copy
from math import *

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *

## detect_obect
#
# In this class the actual object detection is executed exactly one time
# No further actions are executed
class ObjectDetector:
	def __init__(self, detector_srv = '/object_detection/detect_object', object_name = "", min_dist = 2  ):
		self.detector_srv = detector_srv 
		self.object_name = object_name
		self.min_dist = min_dist
		print "Contructor ObjectDetector:",self.detector_srv
	
		print os.environ['PYTHONPATH']

	def execute(self, userdata):
		# determine object name
		if self.object_name != "":
			object_name = self.object_name
		elif type(userdata.object_name) is str:
			object_name = userdata.object_name
		else: # this should never happen
			rospy.logerr("Invalid userdata 'object_name'")
			return 'failed'

		# check if object detection service is available
		try:
			rospy.wait_for_service(self.detector_srv,10)
		except rospy.ROSException, e:
			print "Service not available: %s"%e # no object found within min_dist start value
			return 'failed'

		# call object detection service
		try:
			detector_service = rospy.ServiceProxy(self.detector_srv, DetectObjects)
			req = DetectObjectsRequest()
			req.object_name.data = object_name
			res = detector_service(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

		# HACK TODO FIXME call object detection service TWICE TO GET CURRENT IMAGE
		try:
			detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
			req = DetectObjectsRequest()
			req.object_name.data = object_name
			res = detector_service(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
			
		# check for no objects
		if len(res.object_list.detections) <= 0:
			rospy.logerr("No objects found")
			return 'no_object'
		
		# select nearest object in x-y-plane in head_camera_left_link
		obj = Detection()
		for item in res.object_list.detections:
			dist = sqrt(item.pose.pose.position.x*item.pose.pose.position.x+item.pose.pose.position.y*item.pose.pose.position.y)
			if dist < self.min_dist:
				self.min_dist = dist
				obj = copy.deepcopy(item)
		
		# check if an object could be found within the min_dist start value
		if obj.label == "":
			rospy.logerr("Object not within target range")
			return 'no_object'

		#check if label of object fits to requested object_name
		if obj.label != object_name:
			rospy.logerr("The object name doesn't fit.")
			return 'no_object'

		# HACK for timestamp
		obj.pose.header.stamp = rospy.Time.now()

		# we succeeded to detect an object
		userdata.object = obj
		
		return 'success'
		


## Detect state
#
# This state will try to detect an object in the front of care-o-bot.

class DetectObjectBackside(smach.State):
	def __init__(self,namespace, detector_srv = '/object_detection/detect_object', object_name = "",max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded','no_object','no_more_retries','failed'],
			input_keys=['object_name'],
			output_keys=['object'])

		self.object_list = DetectionArray() # UHR: DO we need that? It is not referenced 
		self.max_retries = max_retries
		self.retries = 0
		self.namespace = namespace

		self.object_detector = ObjectDetector(object_name, detector_srv,2)
	
		#ToDo: Read from yaml	
		
		self.torso_poses = []

		
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

		#self.torso_poses.append("back_right")
		#self.torso_poses.append("back_right_extreme")
		#self.torso_poses.append("back")
		#self.torso_poses.append("back_extreme")
		#self.torso_poses.append("back_left")
		#self.torso_poses.append("back_left_extreme")

	def execute(self, userdata):

		sss.set_light("light", 'blue')
	
		# check if maximum retries reached
		if self.retries > self.max_retries:
			sss.set_light("light", 'yellow')
			self.retries = 0
			handle_torso = sss.move("torso","home",False)
			handle_torso.wait()
			handle_arm = sss.move("arm","look_at_table-to-folded")
			sss.set_light("light", 'blue')
			return 'no_more_retries'
		
		# move sdh as feedback
		sss.move("sdh","cylclosed",False) # UHR: could we choose a different signal? This looks a bit awkward to visitors. We have the speech output anyway - so I would just omit this statement...
		
		# make the robot ready to inspect the scene
		if type(userdata.object_name) is str:
			object_name = userdata.object_name
		else:
			object_name = "given object"

		if self.retries == 0: # only move arm, sdh and head for the first try
			sss.set_light("light", 'yellow')
			sss.say("sound", ["I will now search for the " + object_name + "."],False)
			handle_arm = sss.move("arm","folded-to-look_at_table",False)
			handle_torso = sss.move("torso","shake",False)
			handle_head = sss.move("head","back",False)
			handle_arm.wait()
			handle_head.wait()
			handle_torso.wait()
			sss.set_light("light", 'blue')

		# have an other viewing point for each retry
		handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) 
		
		# move sdh as feedback
		sss.move("sdh","home",False) # UHR: see above
		
		# wait for image to become stable
		sss.sleep(2)
	
		result = self.object_detector.execute(userdata)
		if result == 'failed':
			self.retries = 0
			sss.set_light("light", 'red')
		elif results == "no_object":
			self.retries += 1
		else: #suceeded
			self.retries = 0
			sss.move("torso","home")
		
		return result


## Detect front state
#
# This state will try to detect an object in the back of care-o-bot.
# It encorporates the movement of the arm to ensure that is not in the vision space  of the cameras

class DetectObjectFrontside(smach.State):
	def __init__(self,namespace, detector_srv = '/object_detection/detect_object', object_name = "",max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded','no_object','no_more_retries','failed'],
			input_keys=['object_name'],
			output_keys=['object'])

		self.object_list = DetectionArray() # UHR: Do we need that? It is not referenced in the class ...
		self.max_retries = max_retries
		self.detector_srv = detector_srv
		self.retries = 0
		self.torso_poses = []

		print "DetectObjectFrontside", self.detector_srv
		self.object_detector = ObjectDetector(object_name, self.detector_srv)
	
		#ToDo: Read from yaml	
		if rospy.has_param(namespace):
			params = rospy.get_param(namespace)
			torso_poses = params["torso_poses"]
			for pose in torso_poses:
				if (pose[0] == 'joints'): #(joints,0;0;0)
					self.torso_poses.append((pose[1],pose[2],pose[3]))
				elif (pose[0] == 'xyz'): #(xyz;0;5;3)
					#TODO call look at point in world (MDL)
					print "Call LookAtPointInWorld"
				else: #string:
					self.torso_poses.append(pose)
		else:
			rospy.loginfo("No torso_poses found for state %s on ROS parameter server, taking 'home' as default" %namespace)
			self.torso_poses.append("home") # default pose

		#self.torso_poses = []
		#self.torso_poses.append("front_right")
		#self.torso_poses.append("front_right_extreme")
		#self.torso_poses.append("front")
		#self.torso_poses.append("front_extreme")
		#self.torso_poses.append("front_left")
		#self.torso_poses.append("front_left_extreme")

	def execute(self, userdata):

		sss.set_light("light", 'blue')
	
		# check if maximum retries reached
		if self.retries > self.max_retries:
			sss.set_light("light", 'yellow')
			self.retries = 0
			handle_torso = sss.move("torso","home",False)
			handle_torso.wait()
			handle_arm = sss.move("arm","look_at_table-to-folded")
			sss.set_light("light", 'blue')
			return 'no_more_retries'
		
		# move sdh as feedback
		sss.move("sdh","cylclosed",False) # UHR: could we choose a different signal? This looks a bit awkward to visitors. We have the speech output anyway - so I would just omit this statement...
		
		# make the robot ready to inspect the scene
		if type(userdata.object_name) is str:
			object_name = userdata.object_name
		else:
			object_name = "given object"

		if self.retries == 0: # only move sdh and head for the first try
			sss.set_light("light", 'yellow')
			sss.say("sound", ["I will now search for the " + object_name + "."],False)
			handle_torso = sss.move("torso","shake",False)
			handle_head = sss.move("head","front",False)
			handle_head.wait()
			handle_torso.wait()
			sss.set_light("light", 'blue')

		# have an other viewing point for each retry
		handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) 
		
		# move sdh as feedback
		sss.move("sdh","home",False) # UHR: see above
		
		# wait for image to become stable
		sss.sleep(2)
	
		result = self.object_detector.execute(userdata)
		if result == 'failed':
			self.retries = 0
			sss.set_light("light", 'red')
		elif results == "no_object":
			self.retries += 1
		else: #suceeded
			self.retries = 0
			sss.move("torso","home")
		
		return result
