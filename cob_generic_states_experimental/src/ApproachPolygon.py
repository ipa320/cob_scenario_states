#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import copy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from cob_3d_mapping_msgs.srv import GetApproachPoseForPolygon
import tf
from tf.transformations import *

from ApproachPose import *

"""Computes all accessible robot poses around polygon, returns best pose (successively)"""
class ComputeNavigationGoals(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'failed'],
			input_keys=['polygon','goal_poses','new_computation_flag'],
			output_keys=['goal_poses','new_computation_flag'])
		self.listener = tf.TransformListener()

	def execute(self, userdata):
		if not userdata.new_computation_flag:
			return 'computed'
		rospy.wait_for_service('map_accessibility_analysis/map_polygon_accessibility_check',10)
		try:
			get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_polygon_accessibility_check', GetApproachPoseForPolygon)
			res = get_approach_pose(userdata.polygon)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		userdata.goal_poses = res.approach_poses.poses
		userdata.new_computation_flag = False
		return 'computed'
	
	
class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'no_goals_left', 'failed'],
			input_keys=['polygon','goal_poses', 'goal_pose'],
			output_keys=['goal_pose'])
		self.listener = tf.TransformListener()
		self.nogo_area_radius_squared = 1*1 #in meters, radius the current goal covers

	def execute(self, userdata):
		"""compute closest position to current robot pose"""
		try:
			self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1.5))
			robot_pose = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Could not get robot pose"
			return 'failed'
		closest_pose = Pose()
		minimum_distance_squared = 100000.0
		found_valid_pose = 0
		for pose in userdata.goal_poses:
			dist_squared = (robot_pose[0][0]-pose.position.x)*(robot_pose[0][0]-pose.position.x)+(robot_pose[0][1]-pose.position.y)*(robot_pose[0][1]-pose.position.y)
			if dist_squared < minimum_distance_squared:
				minimum_distance_squared = dist_squared
				closest_pose = copy.deepcopy(pose)
				found_valid_pose = 1
		if found_valid_pose == 0:
			return 'no_goals_left'
		
		"""delete all poses too close to current goal"""
		for p in range(len(userdata.goal_poses)-1,-1,-1):
			pose = userdata.goal_poses[p]
			dist_squared = (closest_pose.position.x-pose.position.x)*(closest_pose.position.x-pose.position.x)+(closest_pose.position.y-pose.position.y)*(closest_pose.position.y-pose.position.y)
			if dist_squared < self.nogo_area_radius_squared:
				userdata.goal_poses.remove(pose)
		
		"""convert goal pose"""
		[roll, pitch, yaw] = euler_from_quaternion([closest_pose.orientation.x,
												closest_pose.orientation.y,
												closest_pose.orientation.z,
												closest_pose.orientation.w])
		userdata.goal_pose=[closest_pose.position.x, closest_pose.position.y, yaw+math.pi]
		return 'computed'



class ApproachPolygon(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['reached', 'not_reached', 'failed'],
			input_keys=['polygon','new_computation_flag'],
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


if __name__ == '__main__':
	rospy.init_node("approach_polygon")
	SM = ApproachPolygon()
	SM.userdata.polygons = []
	#SM.userdata.base_pose = Pose()
	SM.execute()
	rospy.spin()



