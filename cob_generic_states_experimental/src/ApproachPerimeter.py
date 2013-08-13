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
#   Approaches a pose on the perimeter of a circle and checks the accessibility for moving to this pose beforehand with the services provided by cob_map_accessibility_analysis.
#   To use this state machine the map analysis has to be started first: roslaunch cob_map_accessibility_analysis map_accessibility_analysis.launch
#
#   Input keys:
#   'center': Pose2D defining the center point of the circle to be visited. You may provide an orientation angle as well, which defines the "viewing direction" of the target.
#   'radius': Double value of the radius of the circle.
#   'rotational_sampling_step': Double value of the angular sampling step with in [rad] of goal poses on the perimeter of the circle.
#   'goal_pose_selection_strategy': defines which of the possible poses on the circle shall be preferred
#                                   'closest_to_target_gaze_direction' (commands the robot to the pose which is closest to the target's viewing direction, useful e.g. for living targets),
#                                   'closest_to_robot' (commands the robot to the pose closest to the current robot position, useful e.g. for inspecting a location).
#                                   Independent of the mode, this state machine always terminates once a goal position could be reached,
#                                   however, to visit multiple locations on the same circle the state_machine has to be called with 'new_computation_flag' set to False
#                                   and an appropriate 'invalidate_other_poses_radius' until 'not_reached' is returned from 'SELECT_GOAL'
#                                   Internally, the remaining, not yet visited states, are stored in the list 'goal_poses_verified'.
#   'invalidate_other_poses_radius': Within a circle of this radius (in [m]) around the current goal pose all other valid poses become deleted from userdata.goal_poses_verified
#                                    so that the next accessible pose a a certain minimum distance from the current goal pose. 
#   'new_computation_flag': If True, the poses on the defined circle are examined for accessibility, else the old list from userdata.goal_poses_verified is used again.
#                           This variable is used to command the robot to different perspectives on the same goal, which are computed at the first call of this state machine.
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
from cob_map_accessibility_analysis.srv import CheckPerimeterAccessibility
import tf
from tf.transformations import *

from ApproachPose import *

"""Computes all accessible robot poses on perimeter"""
class ComputeNavigationGoals(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'failed'],
			input_keys=['center', 'radius', 'rotational_sampling_step', 'new_computation_flag'],
			output_keys=['goal_poses_verified', 'gaze_direction_goal_pose', 'new_computation_flag'])

	def execute(self, userdata):
		sf = ScreenFormat("ComputeNavigationGoals")
		if not userdata.new_computation_flag:
			return 'computed'
		rospy.wait_for_service('map_accessibility_analysis/map_perimeter_accessibility_check',10)
		try:
			get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_perimeter_accessibility_check', CheckPerimeterAccessibility)
			res = get_approach_pose(userdata.center, userdata.radius, userdata.rotational_sampling_step)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		userdata.goal_poses_verified = res.accessible_poses_on_perimeter
		userdata.new_computation_flag = False
		gaze_direction_goal_pose = Pose2D()
		gaze_direction_goal_pose.x = userdata.center.x + userdata.radius * math.cos(userdata.center.theta)
		gaze_direction_goal_pose.y = userdata.center.y + userdata.radius * math.sin(userdata.center.theta)
		userdata.gaze_direction_goal_pose = gaze_direction_goal_pose
		return 'computed'
	
	
class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'no_goals_left', 'failed'],
			input_keys=['goal_poses_verified', 'gaze_direction_goal_pose', 'goal_pose_selection_strategy', 'invalidate_other_poses_radius'],
			output_keys=['goal_pose'])
		self.listener = tf.TransformListener(True, rospy.Duration(20.0))
		
	def execute(self, userdata):
		sf = ScreenFormat("SelectNavigationGoal")
		
		goal_pose = Pose2D()
		if userdata.goal_pose_selection_strategy=='closest_to_robot':
			"""compute closest position to current robot pose"""
			try:
				t = rospy.Time(0)
				self.listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
				robot_pose = self.listener.lookupTransform('/map', '/base_link', t)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "Could not lookup robot pose: %s" %e
				return 'failed'
			goal_pose.x = robot_pose[0][0]
			goal_pose.y = robot_pose[0][1]
		elif userdata.goal_pose_selection_strategy=='closest_to_target_gaze_direction':
			goal_pose = userdata.gaze_direction_goal_pose
		else:
			print "The selected strategy %s does not match any of the valid choices." %userdata.gaze_direction_goal_pose

		closest_pose = Pose2D()
		minimum_distance_squared = 100000.0
		found_valid_pose = 0
		for pose in userdata.goal_poses_verified:
			dist_squared = (goal_pose.x-pose.x)*(goal_pose.x-pose.x)+(goal_pose.y-pose.y)*(goal_pose.y-pose.y)
			if dist_squared < minimum_distance_squared:
				minimum_distance_squared = dist_squared
				closest_pose = copy.deepcopy(pose)
				found_valid_pose = 1
		if found_valid_pose == 0:
			return 'no_goals_left'
		
		"""delete all poses too close to current goal"""
		nogo_area_radius_squared = userdata.invalidate_other_poses_radius * userdata.invalidate_other_poses_radius
		for p in range(len(userdata.goal_poses_verified)-1,-1,-1):
			pose = userdata.goal_poses_verified[p]
			dist_squared = (closest_pose.x-pose.x)*(closest_pose.x-pose.x)+(closest_pose.y-pose.y)*(closest_pose.y-pose.y)
			if dist_squared < nogo_area_radius_squared:
				userdata.goal_poses_verified.remove(pose)
		
		userdata.goal_pose=[closest_pose.x, closest_pose.y, closest_pose.theta]
		return 'computed'



class ApproachPerimeter(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['reached', 'not_reached', 'failed'],
			input_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_selection_strategy', 'invalidate_other_poses_radius', 'new_computation_flag'],
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
		sm = ApproachPerimeter()
		sm.userdata.center = Pose2D()
		sm.userdata.center.x = -1.0
		sm.userdata.center.y = -1.0
		sm.userdata.center.theta = 0
		sm.userdata.radius = 0.8
		sm.userdata.rotational_sampling_step = 10.0/180.0*math.pi
		sm.userdata.new_computation_flag = True
		sm.userdata.invalidate_other_poses_radius = 1.0 #in meters, radius the current goal covers
		sm.userdata.goal_pose_selection_strategy = 'closest_to_target_gaze_direction'  #'closest_to_target_gaze_direction', 'closest_to_robot'
		
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
