#!/usr/bin/python

# Abstract Class for Defining Skills

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

class SkillsBase(smach.StateMachine):

	__metaclass__ = abc.ABCMeta
    

	@abc.abstractmethod
	def pre_conditions(self):
		return 'Restricted to Abstract Class'

	@abc.abstractmethod
	def post_conditions(self):
		return 'Restricted to Abstract Class'

	@abc.abstractproperty
	def inputs(self):
		return 'Restricted to Abstract Class'
    
	@abc.abstractproperty
	def outputs(self):
		return 'Restricted to Abstract Class'

	@abc.abstractproperty
	def requirements(self):
		return 'Restricted to Abstract Class'

    	
