#!/usr/bin/python

# Example of pre-condition processing

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

from simple_script_server import *  # import script
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *

from abc_conditioncheck import ConditionCheck

import types

class PreConditionCheck(ConditionCheck):

	def __init__(self, checkTypes, tfL):
		smach.State.__init__(self, outcomes=['success','failed'])

		self.checks = checkTypes['pre_check']
		self.robot_name = checkTypes['robot_name']

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

		self.tfL = tfL

		self.result = "failed"

		self.xy_goal_tolerance  = self.checks['pose_check'][0]['allowed_position_error']
		self.yaw_goal_tolerance = self.checks['pose_check'][0]['allowed_orientation_error']



	#####################################################################
	# function: joint_configuration_check()
	# This function is responsible for checking the 
	# Joint configuration of the Robot before performing the skill action
	#####################################################################

	def joint_configuration_check_js(self): # from parameter server, get names and supposed states

		#get joint_names from parameter server (yaml)
		#get joint_states from parameter server (yaml)

		return self.joint_configuration_check(joint_names,joint_states)
 

	def joint_configuration_check_ss(self):

		component = self.checks['joint_configuration_check_ss'][0]['component']
		configuration = self.checks['joint_configuration_check_ss'][0]['configuration']

		ss_names_path = "/script_server/" + component + "/joint_names"
		ss_values_path = "/script_server/" + component + "/" + configuration

		joint_names = rospy.get_param(ss_names_path)
		joint_states = rospy.get_param(ss_values_path)[0]

		return self.joint_configuration_check(joint_names,joint_states)

	def joint_configuration_check(self, joint_names, joint_states):

		joints = zip(joint_names, joint_states)

		try:
			aw_error = self.checks['joint_configuration_check_ss'][0]['allowed_error']
			
			for name, state in joints:

				rospy.loginfo("Checking the <<%s>> joint"%name)
				
				jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

				while name not in jointsMsg.name:
					jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

				value = jointsMsg.position[jointsMsg.name.index(name)]

				assert abs(value - state) <= aw_error, "Error on the Joint Position for the <<%s>>"%name
			
		except AssertionError,e:
			self.result = "failed"
			rospy.logerr("<<Error Message>>:%s"%e)
			rospy.logerr("at:%s"%check)
			return


		self.result = "success"

	####################################################################
	# function: ready_check()
	# This function is responsible for checking the initial conditions
	# provided by the user for proper operation of the Robot
	#####################################################################

	def ready_check(self): 

		# change The checks for using the diagnostics component

		self.init_components()

	
	####################################################################
	# function: pose_check()
	# This function is responsible for checking if the Robot position
	# is convenient for performing the skill action
	####################################################################

	def pose_check(self):
		
		try:
			rospy.loginfo("Checking the Robot Pose")

			self.tfL.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(20.0))

			(trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))

			angles = euler_from_quaternion(rot)
		
			messageX = (str)(trans[0]) + " " + (str)(self.checks['pose_check']['pose'][0]) + " "+ (str)(self.xy_goal_tolerance)
			messageY = (str)(trans[1]) + " " + (str)(self.checks['pose_check']['pose'][1]) + " "+ (str)(self.xy_goal_tolerance)
			messageA = (str)(angles[2]) + " " + (str)(self.checks['pose_check']['pose'][2]) + " "+ (str)(self.yaw_goal_tolerance)


			assert abs(trans[0] - self.checks['pose_check']['pose'][0]) <= self.xy_goal_tolerance, "Error on the X axis position %s"%messageX
			assert abs(trans[1] - self.checks['pose_check']['pose'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position %s"%messageY
			assert abs(angles[2] - self.checks['pose_check']['pose'][2]) <= self.yaw_goal_tolerance, "Error on the Angle %s"%messageA
		
			if self.robot_name == "desire":

				# Checking Arm 7 Link Position 
				if "arm_left" in self.full_components:
					rospy.loginfo("Checking the <<left arm>> Position")
					now = rospy.Time.now()

					self.tfL.waitForTransform('/base_link', '/arm_left_7_link', now, rospy.Duration(4.0))
					(trans,rot) = self.tfL.lookupTransform('/base_link', '/arm_left_7_link', now)

					assert abs(trans[0] - self.checks['pose_check']['links']['arm_left_7_link']['position'][0]) <= self.xy_goal_tolerance, "Error on the X axis position for the ARM 7 Link"
					assert abs(trans[1] - self.checks['pose_check']['links']['arm_left_7_link']['position'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position for the ARM 7 Link"
					assert abs(trans[2] - self.checks['pose_check']['links']['arm_left_7_link']['position'][2]) <= self.xy_goal_tolerance, "Error on the Z axis position for the ARM 7 Link"
					assert abs(rot[0] - self.checks['pose_check']['links']['arm_left_7_link']['position'][3]) <= self.yaw_goal_tolerance, "Error on W for the ARM 7 Link"
					assert abs(rot[1] - self.checks['pose_check']['links']['arm_left_7_link']['position'][4]) <= self.yaw_goal_tolerance, "Error on Raw for the ARM 7 Link"
					assert abs(rot[2] - self.checks['pose_check']['links']['arm_left_7_link']['position'][5]) <= self.yaw_goal_tolerance, "Error on Pitch for the ARM 7 Link"
					assert abs(rot[3] - self.checks['pose_check']['links']['arm_left_7_link']['position'][6]) <= self.yaw_goal_tolerance, "Error on Yaw for the ARM 7 Link"

				if "arm_right" in self.full_components:
					rospy.loginfo("Checking the <<right arm>> Position")
					now = rospy.Time.now()

					self.tfL.waitForTransform('/base_link', '/arm_right_7_link', now, rospy.Duration(4.0))
					(trans,rot) = self.tfL.lookupTransform('/base_link', '/arm_right_7_link', now)

					assert abs(trans[0] - self.checks['pose_check']['links']['arm_right_7_link']['position'][0]) <= self.xy_goal_tolerance, "Error on the X axis position for the ARM 7 Link"
					assert abs(trans[1] - self.checks['pose_check']['links']['arm_right_7_link']['position'][1]) <= self.xy_goal_tolerance, "Error on the Y axis position for the ARM 7 Link"
					assert abs(trans[2] - self.checks['pose_check']['links']['arm_right_7_link']['position'][2]) <= self.xy_goal_tolerance, "Error on the Z axis position for the ARM 7 Link"
					assert abs(rot[0] - self.checks['pose_check']['links']['arm_right_7_link']['position'][3]) <= self.yaw_goal_tolerance, "Error on W for the ARM 7 Link"
					assert abs(rot[1] - self.checks['pose_check']['links']['arm_right_7_link']['position'][4]) <= self.yaw_goal_tolerance, "Error on Raw for the ARM 7 Link"
					assert abs(rot[2] - self.checks['pose_check']['links']['arm_right_7_link']['position'][5]) <= self.yaw_goal_tolerance, "Error on Pitch for the ARM 7 Link"
					assert abs(rot[3] - self.checks['pose_check']['links']['arm_right_7_link']['position'][6]) <= self.yaw_goal_tolerance, "Error on Yaw for the ARM 7 Link"

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

		except AssertionError,e:
			self.result = "failed"
			rospy.logerr("<<Error Message>>:%s"%e)		
			rospy.logerr("at pose_check")
			return
		
		
		self.result = "success" 
	

	####################################################################
	# function: init_components()
	# Initializes robot components according to the components available
	# for the Robot
	####################################################################

	def init_components(self):

		sss.sleep(2)

		if "sound" in self.full_components:
			sss.say(["Preparing."],False)
		# bring robot into the starting state

		if "tray" in self.full_components:
			handle_tray = sss.move("tray","down",False)
			handle_tray.wait()

		if "torso" in self.full_components:
			handle_torso = sss.move("torso","home",False)
			handle_torso.wait()

		if "arm" in self.full_components:
			handle_arm = sss.move("arm","folded",False)
			handle_arm.wait()

		if "arm_left" in self.full_components:
			handle_arm = sss.move("arm_left","folded",False)
			handle_arm.wait()

		if "arm_right" in self.full_components:
			handle_arm = sss.move("arm_right","folded",False)
			handle_arm.wait()


		if "sdh" in self.full_components:
			handle_sdh = sss.move("sdh","cylclosed",False)
			handle_sdh.wait()

		if "head" in self.full_components:
			handle_head = sss.move("head","front",False)
			handle_head.wait()

		# wait for all movements to be finished
		# announce ready
		if "sound" in self.full_components:
			sss.say(["Ready."])

		sss.sleep(2)
		
	####################################################################
	# function: execute()
	# Main routine of the State Machine
	####################################################################

	def execute(self, userdata):	

		if self.checks['ready_check'] == True:
			self.ready_check()

		if (self.result == "failed"):
			rospy.logerr("Check failure on <<ready_check>>")
			return self.result

		for check in self.checks:

				getattr(self, check)()

				if (self.result == "failed"):
					rospy.logerr("Check failure on <<%s>>"%check)
					return self.result

		# announce ready
		if ("sound" in self.full_components) and (self.result == "success"):
			sss.say(["Ready."])

		return self.result
