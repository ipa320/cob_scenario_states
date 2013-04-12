#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from cob_3d_mapping_msgs.srv import GetApproachPoseForPolygon
import tf
from tf.transformations import *

from ApproachPose import *

class ComputeNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'no_goals_left', 'failed'],
			input_keys=['polygon','base_pose'],
			output_keys=['base_pose'])
		self.listener = tf.TransformListener()

	def execute(self, userdata):
		rospy.wait_for_service('compute_approach_pose',10)
		try:
			get_approach_pose = rospy.ServiceProxy('compute_approach_pose', GetApproachPoseForPolygon)
			res = get_approach_pose(userdata.polygon)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		try:
			self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1.5))
			robot_pose = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Could not get robot pose"
			return 'failed'
		closest_pose = Pose()
		minimum_distance_squared = 100000.0
		found_valid_pose = 0
		for pose in res.approach_poses.poses:
			dist_squared = (robot_pose[0][0]-pose.position.x)*(robot_pose[0][0]-pose.position.x)+(robot_pose[0][1]-pose.position.y)*(robot_pose[0][1]-pose.position.y)
			if dist_squared < minimum_distance_squared:
				minimum_distance_squared = dist_squared
				closest_pose = pose
				found_valid_pose = 1
		if found_valid_pose == 0:
			return 'failed'
		[roll, pitch, yaw] = euler_from_quaternion([closest_pose.orientation.x,
												closest_pose.orientation.y,
												closest_pose.orientation.z,
												closest_pose.orientation.w])
		userdata.base_pose=[closest_pose.position.x, closest_pose.position.y, yaw+math.pi]
		return 'computed'



class ApproachPolygon(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['reached', 'not_reached', 'failed'])
		with self:

			smach.StateMachine.add('SELECT_GOAL', ComputeNavigationGoal(),
																transitions={'computed':'MOVE_BASE',
																			'no_goals_left':'not_reached',
																			'failed':'failed'})

			smach.StateMachine.add('MOVE_BASE', ApproachPose(),
																	 transitions={'reached':'reached',
																				'not_reached':'SELECT_GOAL',
																				'failed':'failed'})


if __name__ == '__main__':
	rospy.init_node("approach_polygon")
	SM = ApproachPolygon()
	SM.userdata.polygons = []
	#SM.userdata.base_pose = Pose()
	SM.execute()
	rospy.spin()



