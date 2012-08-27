#!/usr/bin/python

# Example of post condition processing 

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
import tf
from tf.msg import tfMessage 
from tf.transformations import euler_from_quaternion

from simple_script_server import *
sss = simple_script_server()	


from abc_conditioncheck import ConditionCheck

class PostConditionCheck(ConditionCheck):

	def __init__(self, checkTypes):
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['base_pose'])

		self.checks = checkTypes['post_check']

		self.result = "failed"

		self.tfL = tf.TransformListener()

		self.xy_goal_tolerance  = self.checks['pose_check']['allowed_error_position']
		self.yaw_goal_tolerance = self.checks['pose_check']['allowed_error_orientation']


	#####################################################################
	# function: joint_configuration_check()
	# This function is responsible for checking the 
	# Joint configuration of the Robot after performing the skill action
	#####################################################################

	def joint_configuration_check(self, userdata):

		def arm_left_joint():
			print "ARM LEFT!"
			

		def arm_right_joint():
			print "ARM RIGHT!"
			

		def arm_joint():
			print "ARM JOINT!"
			

		def torso_joint():
			print "TORSO JOINT!"
			

		for check in self.checks['joint_configuration_check']:

			if self.checks['robot_name'] == "desire":
				try:
					{'arm_left': arm_left_joint, 'arm_right': arm_right_joint, 'torso': torso_joint}[check]()

				except KeyError:
					pass
			else:
				try:
					{'arm': arm_joint, 'torso': torso_joint}[check]()

				except KeyError:
					pass

	####################################################################
	# function: pose_check()
	# This function is responsible for checking if the Robot position
	# is convenient with the result from performing the skill action
	####################################################################

	def pose_check(self, userdata):

		start_time = rospy.rostime.get_time()

		self.tfL.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(20.0))

		(trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))

		angles = euler_from_quaternion(rot)
		
		messageX = (str)(trans[0]) + " " + (str)(userdata.base_pose[0]) + " "+ (str)(self.xy_goal_tolerance)
		messageY = (str)(trans[1]) + " " + (str)(userdata.base_pose[1]) + " "+ (str)(self.xy_goal_tolerance)
		messageA = (str)(angles[2]) + " " + (str)(userdata.base_pose[2]) + " "+ (str)(self.yaw_goal_tolerance)
		
		print messageX, messageY, messageA

		assert abs(trans[0] - userdata.base_pose[0]) <= self.xy_goal_tolerance, "Error on the X axis position %s"%messageX
		assert abs(trans[1] - userdata.base_pose[1]) <= self.xy_goal_tolerance, "Error on the Y axis position %s"%messageY
		assert abs(angles[2] - userdata.base_pose[2]) <= self.yaw_goal_tolerance, "Error on the Angle %s"%messageA

		self.result = "success" 

	####################################################################
	# function: execute()
	# Main routine of the State Machine
	####################################################################

	def execute(self, userdata):

		for check in self.checks:
			try:
				{'joint_configuration_check': self.joint_configuration_check, 'pose_check': self.pose_check}[check](userdata)

			except KeyError:
				pass
		
		return self.result
