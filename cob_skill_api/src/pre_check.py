#!/usr/bin/python

# Example of pre-condition processing

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *

from simple_script_server import *  # import script
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *

from abc_conditioncheck import ConditionCheck

class PreConditionCheck(ConditionCheck):

	def __init__(self, checkTypes):
		smach.State.__init__(self, outcomes=['success','failed'])

		self.checks = checkTypes['pre_check']

		self.result = "failed"

	####################################################################
	# function: init_check()
	# This function is responsible for checking the initial conditions
	# provided by the user for proper operation of the Robot
	#####################################################################

	def init_check(self):

		def init_base():
			print "INIT BASE!"
			

		def init_head():
			handle_head = sss.move("head","front",False)
			handle_head.wait()
			self.result = "success"

		def init_arm():
			handle_arm = sss.move("arm","look_at_table-to-folded",False)
			handle_arm.wait()
			self.result = "success"

		def init_tray():
			handle_tray = sss.move("tray","down",False)
			handle_tray.wait()
			print "Does work!"
			self.result = "success"

		def init_torso():

			handle_torso = sss.move("torso","home",False)
			handle_torso.wait()
			print "Does work!"
			self.result = "success"

		def init_sdh():
			handle_sdh = sss.move("sdh","cylclosed",False)
			handle_sdh.wait()
			self.result = "success"

		for check in self.checks['init_check']:
			try:
				{'base': init_base, 'tray': init_tray, 'torso': init_torso}[check]()

			except KeyError:
				pass

	#####################################################################
	# function: joint_configuration_check()
	# This function is responsible for checking the 
	# Joint configuration of the Robot before performing the skill action
	#####################################################################

	def joint_configuration_check(self):


		def arm_left_joint():

			aw_error = self.checks['joint_configuration_check']['arm_left']['allowed_error']

			seed_js = JointState()
                	seed_js.name = rospy.get_param("/arm_left_controller/joint_names")
                	seed_js.position = rospy.get_param("/script_server/arm_left/intermediatefront")[0]

			print "ARM LEFT!"
			

		def arm_right_joint():
			aw_error = self.checks['joint_configuration_check']['arm_right']['allowed_error']
			print aw_error, "ARM RIGHT!"
			

		def arm_joint():
			aw_error = self.checks['joint_configuration_check']['arm']['allowed_error']

			seed_js = JointState()
                	seed_js.name = rospy.get_param("/arm_controller/joint_names")
                	seed_js.position = rospy.get_param("/script_server/arm/intermediatefront")[0]

			print "ARM JOINT!"
			

		def torso_joint():
			aw_error = self.checks['joint_configuration_check']['torso']['allowed_error']
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
	# is convenient for performing the skill action
	####################################################################

	def pose_check(self):
		print "CHECK POSE!"
		

	####################################################################
	# function: execute()
	# Main routine of the State Machine
	####################################################################

	def execute(self, userdata):	

		sss.sleep(2)

		sss.say(["Preparing."],False)	

		for check in self.checks:
			try:
				{'init_check': self.init_check, 'joint_configuration_check': self.joint_configuration_check, 'pose_check': self.pose_check}[check]()

			except KeyError:
				pass

		# announce ready
		sss.say(["Ready."])

		return self.result
