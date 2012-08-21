#!/usr/bin/python

# Example of post condition processing 

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib.msg import *

from abc_conditioncheck import ConditionCheck

class PostConditionCheck(ConditionCheck):

	def __init__(self, checks):
		smach.State.__init__(self, outcomes=['success','failed'])
		self.checks = checks

	def execute(self, userdata):
		for check in self.checks:
                        if not (lambda check: check == "success"):
                                return 'failed'
                        return 'success'
