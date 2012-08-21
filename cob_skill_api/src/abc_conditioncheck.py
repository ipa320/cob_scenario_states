#!/usr/bin/python

# Abstract class for condition checking 

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib.msg import *
import abc

class ConditionCheck(smach.State):

	__metaclass__ = abc.ABCMeta

	@abc.abstractmethod
	def execute(self, userdata):
		return "Condition Check needs an Execute Method"
