#!/usr/bin/python

#################################################################
##\file
#
# \note
#   Copyright (c) 2013 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_scenario_states
# \note
#   ROS package name: cob_generic_states_experimental
#
# \author
#   Richard Bormann, email:richard.bormann@ipa.fraunhofer.de
#
# \date Date of creation: August 2013
#
# \brief
#   Approaches a pose or a list of poses and checks the accessibility for moving to these poses beforehand with the services provided by cob_map_accessibility_analysis.
#   To use this state machine the map analysis has to be started first: roslaunch cob_map_accessibility_analysis map_accessibility_analysis.launch
#
#   Input keys:
#   'goal_poses': array of geometry_msgs/Pose2D objects, each describing a goal position of the robot
#   'goal_pose_application': defines the mode of usage of the provided goal poses
#                            'visit_all_in_order' (commands the robot to all poses in the provided order),
#                            'visit_all_nearest' (commands the robot to all poses using the closest next pose each time),
#                            'use_as_alternatives' (visits the first pose of the list that is reachable)
#                            Independent of the mode, this state machine always terminates once a goal position could be reached,
#                            so for the visit_all modes the state_machine has to be called until 'not_reached' is returned from 'SELECT_GOAL'
#                            Internally, the remaining, not yet visited states, are stored in the list 'goal_poses_verified' for the visit_all modes.
#   'new_computation_flag': If True, the provided list of poses is examined for accessibility, else the old list from userdata.goal_poses_verified is used again.
#                           This variable is used by the visit_all modes, which work on the already existing list of goal poses after the first call.
#
#   Output_keys:
#   'new_computation_flag': see above
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
import copy
import smach
import smach_ros
from ScreenFormatting import *

#from simple_script_server import *  # import script
#sss = simple_script_server()

from geometry_msgs.msg import Pose2D
from cob_map_accessibility_analysis.srv import CheckPointAccessibility
import tf
from tf.transformations import *

from ApproachPose import *

"""Computes all accessible robot poses on perimeter"""
class ComputeNavigationGoals(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'failed'],
			input_keys=['goal_poses', 'goal_pose_application', 'new_computation_flag'],
			output_keys=['goal_poses_verified', 'new_computation_flag'])

	def execute(self, userdata):
		sf = ScreenFormat("ComputeNavigationGoals")
		if not userdata.new_computation_flag:
			return 'computed'
		rospy.wait_for_service('map_accessibility_analysis/map_points_accessibility_check',10)
		try:
			get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_points_accessibility_check', CheckPointAccessibility)
			res = get_approach_pose(userdata.goal_poses)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		goal_poses_verified = []
		for i in range(len(userdata.goal_poses)):
			if res.accessibility_flags[i] == True:
				goal_poses_verified.append(userdata.goal_poses[i])
		userdata.goal_poses_verified = goal_poses_verified
		
		if userdata.goal_pose_application=='visit_all_in_order' or userdata.goal_pose_application=='visit_all_nearest':
			userdata.new_computation_flag = False
		elif userdata.goal_pose_application=='use_as_alternatives':
			userdata.new_computation_flag = True
		else:
			print "The selected goal_pose_application %s does not match any of the valid choices." %userdata.goal_pose_application
		return 'computed'
	
	
class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'no_goals_left', 'failed'],
			input_keys=['goal_poses_verified', 'goal_pose_application'],
			output_keys=['goal_pose'])
		self.listener = tf.TransformListener(True, rospy.Duration(20.0))
		self.nogo_area_radius_squared = 0*0 #in meters, radius the current goal covers
		
	def execute(self, userdata):
		sf = ScreenFormat("SelectNavigationGoal")
		
		goal_pose = Pose2D()
		if userdata.goal_pose_application=='visit_all_in_order' or userdata.goal_pose_application=='use_as_alternatives':
			""" use next pose in given order"""
			if len(userdata.goal_poses_verified)>0:
				goal_pose = userdata.goal_poses_verified[0]
			else:
				return 'no_goals_left'
		elif userdata.goal_pose_application=='visit_all_nearest':
			"""compute closest position to current robot pose"""
			try:
				t = rospy.Time(0)
				self.listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
				robot_pose = self.listener.lookupTransform('/map', '/base_link', t)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "Could not lookup robot pose: %s" %e
				return 'failed'

			# todo: also obey rotation for closeness measure
			minimum_distance_squared = 100000.0
			found_valid_pose = 0
			for pose in userdata.goal_poses_verified:
				dist_squared = (robot_pose[0][0]-pose.x)*(robot_pose[0][0]-pose.x)+(robot_pose[0][1]-pose.y)*(robot_pose[0][1]-pose.y)
				if dist_squared < minimum_distance_squared:
					minimum_distance_squared = dist_squared
					goal_pose = copy.deepcopy(pose)
					found_valid_pose = 1
			if found_valid_pose == 0:
				return 'no_goals_left'
		else:
			print "The selected goal_pose_application %s does not match any of the valid choices." %userdata.goal_pose_application
			
 		"""delete the current goal from the list of goal poses"""
 		for p in range(len(userdata.goal_poses_verified)-1,-1,-1):
 			pose = userdata.goal_poses_verified[p]
 			if pose == goal_pose:
 				userdata.goal_poses_verified.remove(pose)
		
# 		"""delete all poses too close to current goal"""
# 		for p in range(len(userdata.goal_poses_verified)-1,-1,-1):
# 			pose = userdata.goal_poses_verified[p]
# 			dist_squared = (goal_pose.x-pose.x)*(goal_pose.x-pose.x)+(goal_pose.y-pose.y)*(goal_pose.y-pose.y)
# 			if dist_squared < self.nogo_area_radius_squared:
# 				userdata.goal_poses_verified.remove(pose)
		
		userdata.goal_pose=[goal_pose.x, goal_pose.y, goal_pose.theta]
		return 'computed'



class ApproachPoses(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['reached', 'not_reached', 'failed'],
			input_keys=['goal_poses', 'goal_pose_application', 'new_computation_flag'],
			output_keys=['new_computation_flag'])
		with self:

			smach.StateMachine.add('COMPUTE_GOALS', ComputeNavigationGoals(),
						transitions={'computed':'SELECT_GOAL',
									'failed':'failed'})
			
			smach.StateMachine.add('SELECT_GOAL', SelectNavigationGoal(),
						transitions={'computed':'MOVE_BASE',
									'no_goals_left':'not_reached',
									'failed':'failed'})

			smach.StateMachine.add('MOVE_BASE', ApproachPose(),
						transitions={'reached':'reached',
									'not_reached':'SELECT_GOAL',
									'failed':'failed'},
						remapping = {'base_pose':'goal_pose'})


"""exemplary usage of this state machine"""

if __name__ == '__main__':
	try:
		rospy.init_node("approach_perimeter")
		sm = ApproachPoses()
		sm.userdata.goal_poses = []
		pose = Pose2D()
		pose.x = -1.5
		pose.y = 0.0
		pose.theta = 0.0
		sm.userdata.goal_poses.append(pose)
		pose = Pose2D()
		pose.x = 0.0
		pose.y = 0.0
		pose.theta = 0.0
		sm.userdata.goal_poses.append(pose)
		pose = Pose2D()
		pose.x = 1.0
		pose.y = 0.0
		pose.theta = math.pi/4.0
		sm.userdata.goal_poses.append(pose)
		pose = Pose2D()
		pose.x = 2.0
		pose.y = -1.0
		pose.theta = math.pi
		sm.userdata.goal_poses.append(pose)
		sm.userdata.new_computation_flag = True
		sm.userdata.goal_pose_application = 'use_as_alternatives' # 'visit_all_in_order' (commands the robot to all poses in the provided order), 'visit_all_nearest' (commands the robot to all poses using the closest next pose each time), 'use_as_alternatives' (visits the first pose of the list that is reachable)
		
		# introspection -> smach_viewer
		sis = smach_ros.IntrospectionServer('map_accessibility_analysis_introspection', sm, '/MAP_ACCESSIBILITY_ANALYSIS')
		sis.start()
		
		sm.execute()
		rospy.spin()
		sis.stop()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)