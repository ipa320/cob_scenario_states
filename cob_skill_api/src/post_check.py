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

		self.checks = checkTypes
		self.checks = self.checks.split()

		self.result = "failed"

		self.tfL = tf.TransformListener()

		self.xy_goal_tolerance  = 0.3
		self.yaw_goal_tolerance = 0.5

	def execute(self, userdata):

		def check_MoveBase():

			start_time = rospy.rostime.get_time()

			self.tfL.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(20.0))

			(trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))
	
			angles = euler_from_quaternion(rot)
			
			messageX = (str)(trans[0]) + " " + (str)(userdata.base_pose[0]) + " "+ (str)(self.xy_goal_tolerance)
			messageY = (str)(trans[1]) + " " + (str)(userdata.base_pose[1]) + " "+ (str)(self.xy_goal_tolerance)
			messageA = (str)(rot[2]) + " " + (str)(userdata.base_pose[2]) + " "+ (str)(self.yaw_goal_tolerance)
			
			print messageX, messageY, messageA

			assert abs(trans[0] - userdata.base_pose[0]) <= self.xy_goal_tolerance, "Error on the X axis position %s"%messageX
			assert abs(trans[1] - userdata.base_pose[1]) <= self.xy_goal_tolerance, "Error on the Y axis position %s"%messageY
			assert abs(angles[2] - userdata.base_pose[2]) <= self.yaw_goal_tolerance, "Error on the Angle %s"%messageA

			self.result = "success" 

		def check_ExampleType():
			print "Does work!"
			self.result = "success" 

		for check in self.checks:
			try:
				{'MoveBase': check_MoveBase, 'ExampleType': check_ExampleType}[check]()

			except KeyError:
				pass
		
		return self.result
