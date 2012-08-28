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

	def __init__(self, checkTypes, tfL):
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['base_pose'])

		self.checks = checkTypes['post_check']
		self.robot_name = checkTypes['robot_name']

		self.result = "failed"

		self.tfL = tfL

		self.required_components = checkTypes['required_components']
		self.optional_components = checkTypes['optional_components']
		
		if isinstance(self.required_components, types.NoneType) and isinstance(self.optional_components, types.NoneType):
			self.full_components = ""
			self.optional_components = ""
			self.required_components = ""
		
		elif isinstance(self.required_components, types.NoneType):
			self.full_components = self.optional_components 
			self.required_components = ""

		elif isinstance(self.optional_components, types.NoneType):
			self.full_components = self.required_components 
			self.optional_components = ""
		else:
			self.full_components = self.required_components + " " + self.optional_components 
		
		self.required_components = self.required_components.split()
		self.optional_components = self.optional_components.split()
		self.full_components = self.full_components.split()

		self.xy_goal_tolerance  = self.checks['pose_check']['allowed_error_position']
		self.yaw_goal_tolerance = self.checks['pose_check']['allowed_error_orientation']


	#####################################################################
	# function: joint_configuration_check()
	# This function is responsible for checking the 
	# Joint configuration of the Robot after performing the skill action
	#####################################################################

	def joint_configuration_check(self, userdata):

		def arm_left_joint():

			rospy.loginfo("Checking the left <<arm>> joints")
			arm_left_joints = rospy.get_param("/script_server/arm_left/joint_names")
			jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

			while arm_left_joints[0] not in jointsMsg.name:
				jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
			    # message is from arm

			angles = []
			for name in arm_left_joints:
				angles.append(jointsMsg.position[jointsMsg.name.index(name)])

			aw_error = self.checks['joint_configuration_check']['arm_left']['allowed_error']

			assert abs(angles[0] - self.checks['joint_configuration_check']['arm_left']['joint_states'][0]) <= aw_error, "Error on the Left Arm for the 1-DOF"
			assert abs(angles[1] - self.checks['joint_configuration_check']['arm_left']['joint_states'][1]) <= aw_error, "Error on the Left Arm for the 2-DOF"
			assert abs(angles[2] - self.checks['joint_configuration_check']['arm_left']['joint_states'][2]) <= aw_error, "Error on the Left Arm for the 3-DOF"
			assert abs(angles[3] - self.checks['joint_configuration_check']['arm_left']['joint_states'][3]) <= aw_error, "Error on the Left Arm for the 4-DOF"
			assert abs(angles[4] - self.checks['joint_configuration_check']['arm_left']['joint_states'][4]) <= aw_error, "Error on the Left Arm for the 5-DOF"
			assert abs(angles[5] - self.checks['joint_configuration_check']['arm_left']['joint_states'][5]) <= aw_error, "Error on the Left Arm for the 6-DOF"
			assert abs(angles[6] - self.checks['joint_configuration_check']['arm_left']['joint_states'][6]) <= aw_error, "Error on the Left Arm for the 7-DOF"


		def arm_right_joint():

			rospy.loginfo("Checking the right <<arm>> joints")
			arm_right_joints = rospy.get_param("/script_server/arm_right/joint_names")
			jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

			while arm_right_joints[0] not in jointsMsg.name:
				jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
			    # message is from arm

			angles = []
			for name in arm_right_joints:
				angles.append(jointsMsg.position[jointsMsg.name.index(name)])

			aw_error = self.checks['joint_configuration_check']['arm_right']['allowed_error']

			assert abs(angles[0] - self.checks['joint_configuration_check']['arm_right']['joint_states'][0]) <= aw_error, "Error on the right Arm for the 1-DOF"
			assert abs(angles[1] - self.checks['joint_configuration_check']['arm_right']['joint_states'][1]) <= aw_error, "Error on the right Arm for the 2-DOF"
			assert abs(angles[2] - self.checks['joint_configuration_check']['arm_right']['joint_states'][2]) <= aw_error, "Error on the right Arm for the 3-DOF"
			assert abs(angles[3] - self.checks['joint_configuration_check']['arm_right']['joint_states'][3]) <= aw_error, "Error on the right Arm for the 4-DOF"
			assert abs(angles[4] - self.checks['joint_configuration_check']['arm_right']['joint_states'][4]) <= aw_error, "Error on the right Arm for the 5-DOF"
			assert abs(angles[5] - self.checks['joint_configuration_check']['arm_right']['joint_states'][5]) <= aw_error, "Error on the right Arm for the 6-DOF"
			assert abs(angles[6] - self.checks['joint_configuration_check']['arm_right']['joint_states'][6]) <= aw_error, "Error on the right Arm for the 7-DOF"
			

		def arm_joint():
			arm_joints = rospy.get_param("/script_server/arm/joint_names")
			jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

			while arm_joints[0] not in jointsMsg.name:
				jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
			    # message is from arm

			angles = []
			for name in arm_joints:
				angles.append(jointsMsg.position[jointsMsg.name.index(name)])

			aw_error = self.checks['joint_configuration_check']['arm']['allowed_error']

			assert abs(angles[0] - self.checks['joint_configuration_check']['arm']['joint_states'][0]) <= aw_error, "Error on the Arm for the 1-DOF"
			assert abs(angles[1] - self.checks['joint_configuration_check']['arm']['joint_states'][1]) <= aw_error, "Error on the Arm for the 2-DOF"
			assert abs(angles[2] - self.checks['joint_configuration_check']['arm']['joint_states'][2]) <= aw_error, "Error on the Arm for the 3-DOF"
			assert abs(angles[3] - self.checks['joint_configuration_check']['arm']['joint_states'][3]) <= aw_error, "Error on the Arm for the 4-DOF"
			assert abs(angles[4] - self.checks['joint_configuration_check']['arm']['joint_states'][4]) <= aw_error, "Error on the Arm for the 5-DOF"
			assert abs(angles[5] - self.checks['joint_configuration_check']['arm']['joint_states'][5]) <= aw_error, "Error on the Arm for the 6-DOF"
			assert abs(angles[6] - self.checks['joint_configuration_check']['arm']['joint_states'][6]) <= aw_error, "Error on the Arm for the 7-DOF"


		def torso_joint():
			aw_error = self.checks['joint_configuration_check']['torso']['allowed_error']

			torso_joints = rospy.get_param("/script_server/torso/joint_names")
			jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

			while torso_joints[0] not in jointsMsg.name:
				jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
			    # message is from torso

			angles = []
			for name in torso_joints:
				angles.append(jointsMsg.position[jointsMsg.name.index(name)])

			aw_error = self.checks['joint_configuration_check']['torso']['allowed_error']
	
			assert abs(angles[0] - self.checks['joint_configuration_check']['torso']['joint_states'][0]) <= aw_error, "Error on the Torso for the 1-DOF"
			assert abs(angles[1] - self.checks['joint_configuration_check']['torso']['joint_states'][1]) <= aw_error, "Error on the Torso for the 2-DOF"
			assert abs(angles[2] - self.checks['joint_configuration_check']['torso']['joint_states'][2]) <= aw_error, "Error on the Torso for the 3-DOF"
			

		for check in self.checks['joint_configuration_check']:

			if self.robot_name == "desire":
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

		rospy.loginfo("Checking the Robot Pose")

		start_time = rospy.rostime.get_time()

		self.tfL.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(20.0))

		(trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))

		angles = euler_from_quaternion(rot)
		
		messageX = (str)(trans[0]) + " " + (str)(userdata.base_pose[0]) + " "+ (str)(self.xy_goal_tolerance)
		messageY = (str)(trans[1]) + " " + (str)(userdata.base_pose[1]) + " "+ (str)(self.xy_goal_tolerance)
		messageA = (str)(angles[2]) + " " + (str)(userdata.base_pose[2]) + " "+ (str)(self.yaw_goal_tolerance)


		assert abs(trans[0] - userdata.base_pose[0]) <= self.xy_goal_tolerance, "Error on the X axis position %s"%messageX
		assert abs(trans[1] - userdata.base_pose[1]) <= self.xy_goal_tolerance, "Error on the Y axis position %s"%messageY
		assert abs(angles[2] - userdata.base_pose[2]) <= self.yaw_goal_tolerance, "Error on the Angle %s"%messageA

		if self.robot_name == "desire":

			# Checking Arm 7 Link Position 
			if "arm_left" in self.full_components:
				rospy.loginfo("Checking the <<left arm>> Position")
				now = rospy.Time.now()

				self.tfL.waitForTransform('/base_link', '/arm_left_7_link', now, rospy.Duration(4.0))
				(trans,rot) = self.tfL.lookupTransform('/base_link', '/arm_left_7_link', now)

				assert abs(trans[0] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][0]) <= self.xy_goal_tolerance, "Error on the X axis position for the ARM 7 Link"
				assert abs(trans[1] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position for the ARM 7 Link"
				assert abs(trans[2] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][2]) <= self.xy_goal_tolerance, "Error on the Z axis position for the ARM 7 Link"
				assert abs(rot[0] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][3]) <= self.yaw_goal_tolerance, "Error on W for the ARM 7 Link"
				assert abs(rot[1] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][4]) <= self.yaw_goal_tolerance, "Error on Raw for the ARM 7 Link"
				assert abs(rot[2] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][5]) <= self.yaw_goal_tolerance, "Error on Pitch for the ARM 7 Link"
				assert abs(rot[3] - self.checks['pose_check']['links']['/arm_left_7_link']['position'][6]) <= self.yaw_goal_tolerance, "Error on Yaw for the ARM 7 Link"

			if "arm_right" in self.full_components:
				rospy.loginfo("Checking the <<right arm>> Position")
				now = rospy.Time.now()

				self.tfL.waitForTransform('/base_link', '/arm_right_7_link', now, rospy.Duration(4.0))
				(trans,rot) = self.tfL.lookupTransform('/base_link', '/arm_right_7_link', now)

				assert abs(trans[0] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][0]) <= self.xy_goal_tolerance, "Error on the X axis position for the ARM 7 Link"
				assert abs(trans[1] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position for the ARM 7 Link"
				assert abs(trans[2] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][2]) <= self.xy_goal_tolerance, "Error on the Z axis position for the ARM 7 Link"
				assert abs(rot[0] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][3]) <= self.yaw_goal_tolerance, "Error on W for the ARM 7 Link"
				assert abs(rot[1] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][4]) <= self.yaw_goal_tolerance, "Error on Raw for the ARM 7 Link"
				assert abs(rot[2] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][5]) <= self.yaw_goal_tolerance, "Error on Pitch for the ARM 7 Link"
				assert abs(rot[3] - self.checks['pose_check']['links']['/arm_right_7_link']['position'][6]) <= self.yaw_goal_tolerance, "Error on Yaw for the ARM 7 Link"

		else:
			# Checking Arm 7 Link Position 
			if "arm" in self.full_components:
				rospy.loginfo("Checking the <<arm>> Position")
				now = rospy.Time.now()

				self.tfL.waitForTransform('/base_link', '/arm_7_link', now, rospy.Duration(4.0))
				(trans,rot) = self.tfL.lookupTransform('/base_link', '/arm_7_link', now)

				assert abs(trans[0] - self.checks['pose_check']['links']['arm_7_link']['position'][0]) <= self.xy_goal_tolerance, "Error on the X axis position for the ARM 7 Link"
				assert abs(trans[1] - self.checks['pose_check']['links']['arm_7_link']['position'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position for the ARM 7 Link"
				assert abs(trans[2] - self.checks['pose_check']['links']['arm_7_link']['position'][2]) <= self.xy_goal_tolerance, "Error on the Z axis position for the ARM 7 Link"
				assert abs(rot[0] - self.checks['pose_check']['links']['arm_7_link']['position'][3]) <= self.yaw_goal_tolerance, "Error on W for the ARM 7 Link"
				assert abs(rot[1] - self.checks['pose_check']['links']['arm_7_link']['position'][4]) <= self.yaw_goal_tolerance, "Error on Raw for the ARM 7 Link"
				assert abs(rot[2] - self.checks['pose_check']['links']['arm_7_link']['position'][5]) <= self.yaw_goal_tolerance, "Error on Pitch for the ARM 7 Link"
				assert abs(rot[3] - self.checks['pose_check']['links']['arm_7_link']['position'][6]) <= self.yaw_goal_tolerance, "Error on Yaw for the ARM 7 Link"

		# Checking Head Axis Link Position 
		if "head" in self.full_components:
			rospy.loginfo("Checking the <<head>> Position")
			now = rospy.Time.now()

			self.tfL.waitForTransform('/base_link', '/head_axis_link', now, rospy.Duration(4.0))
			(trans,rot) = self.tfL.lookupTransform('/base_link', '/head_axis_link', now)
		
			assert abs(trans[0] - self.checks['pose_check']['links']['head_axis_link']['position'][0]) <= self.xy_goal_tolerance, "Error on the X axis position for the Head Axis Link"
			assert abs(trans[1] - self.checks['pose_check']['links']['head_axis_link']['position'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position for the Head Axis Link"
			assert abs(trans[2] - self.checks['pose_check']['links']['head_axis_link']['position'][2]) <= self.xy_goal_tolerance, "Error on the Z axis position for the Head Axis Link"
			assert abs(rot[0] - self.checks['pose_check']['links']['head_axis_link']['position'][3]) <= self.yaw_goal_tolerance, "Error on W for the Head Axis Link"
			assert abs(rot[1] - self.checks['pose_check']['links']['head_axis_link']['position'][4]) <= self.yaw_goal_tolerance, "Error on Raw for the Head Axis Link"
			assert abs(rot[2] - self.checks['pose_check']['links']['head_axis_link']['position'][5]) <= self.yaw_goal_tolerance, "Error on Pitch for the Head Axis Link"
			assert abs(rot[3] - self.checks['pose_check']['links']['head_axis_link']['position'][6]) <= self.yaw_goal_tolerance, "Error on Yaw for the Head Axis Link"
		

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
