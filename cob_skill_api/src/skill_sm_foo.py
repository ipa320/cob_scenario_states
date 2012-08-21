#!/usr/bin/python

# Sample Implementation of a state machine

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib.msg import *

from abc_sm_skill import SkillsSM

class skill_sm_foo(SkillsSM):
	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['success','failed'])

	def execute(self, userdata):
		with self:
			return "success"
