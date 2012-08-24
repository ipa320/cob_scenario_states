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

from abc_conditioncheck import ConditionCheck

class PreConditionCheck(ConditionCheck):

	def __init__(self, checkTypes):
		smach.State.__init__(self, outcomes=['success','failed'])

		self.checks = checkTypes
		self.checks = self.checks.split()

		self.result = "failed"

	def execute(self, userdata):		

		def check_Ex1():
			print "Does work!"
			self.result = "success" 

		def check_Ex2():
			print "Does work!"
			self.result = "success" 

		def check_robotPrep():

			sss.sleep(2)

			sss.say(["Preparing."],False)
		
			# bring robot into the starting state
			handle_tray = sss.move("tray","down",False)
			handle_torso = sss.move("torso","home",False)
			handle_arm = sss.move("arm","look_at_table-to-folded",False)
			handle_sdh = sss.move("sdh","cylclosed",False)
			handle_head = sss.move("head","front",False)
		
			# wait for all movements to be finished
			handle_tray.wait()
			handle_torso.wait()
			handle_arm.wait()
			handle_sdh.wait()
			handle_head.wait()
		
		
			# announce ready
			sss.say(["Ready."])

			self.result = "success"

		for check in self.checks:
			try:
				{'Ex1': check_Ex1, 'Ex2': check_Ex2, 'PrepareRobot': check_robotPrep}[check]()

			except KeyError:
				pass

		return self.result
