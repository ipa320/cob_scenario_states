#!/usr/bin/python
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
			output_keys=['goal_poses', 'gaze_direction_goal_pose', 'new_computation_flag'])

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
		userdata.goal_poses = res.accessible_poses_on_perimeter
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
			input_keys=['goal_poses', 'gaze_direction_goal_pose', 'goal_pose_selection_strategy'],
			output_keys=['goal_pose'])
		self.listener = tf.TransformListener(True, rospy.Duration(20.0))
		self.nogo_area_radius_squared = 1*1 #in meters, radius the current goal covers
		
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
		for pose in userdata.goal_poses:
			dist_squared = (goal_pose.x-pose.x)*(goal_pose.x-pose.x)+(goal_pose.y-pose.y)*(goal_pose.y-pose.y)
			if dist_squared < minimum_distance_squared:
				minimum_distance_squared = dist_squared
				closest_pose = copy.deepcopy(pose)
				found_valid_pose = 1
		if found_valid_pose == 0:
			return 'no_goals_left'
		
		"""delete all poses too close to current goal"""
		for p in range(len(userdata.goal_poses)-1,-1,-1):
			pose = userdata.goal_poses[p]
			dist_squared = (closest_pose.x-pose.x)*(closest_pose.x-pose.x)+(closest_pose.y-pose.y)*(closest_pose.y-pose.y)
			if dist_squared < self.nogo_area_radius_squared:
				userdata.goal_poses.remove(pose)
		
		userdata.goal_pose=[closest_pose.x, closest_pose.y, closest_pose.theta]
		return 'computed'



class ApproachPerimeter(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['reached', 'not_reached', 'failed'],
			input_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_selection_strategy', 'new_computation_flag'],
			output_keys=[])
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