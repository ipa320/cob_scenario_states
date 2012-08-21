#!/usr/bin/python

# Abstract Class for Defining Skills State Machines

import abc

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib.msg import *

import smach

class SkillsSM(smach.StateMachine):

	__metaclass__ = abc.ABCMeta
    

	@abc.abstractmethod
	def execute(self, userdata):
		return 'State Machine needs to be executed'

