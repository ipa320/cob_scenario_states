#!/usr/bin/python

# Sample Implementation of the Abstract Class Skill

#!/usr/bin/python

import abc
from abc_skill import SkillsBase
import yaml

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib.msg import *

import pre_check
import post_check


import skill_sm_foo

class SkillImplementation(SkillsBase):

	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['success', 'failed', 'ended'])
		with self:
			self.add('PRECONDITION_CHECK', self.pre_conditions("yaml/pre.yaml"), transitions={'success':'SKILL_SM'})
			self.add('SKILL_SM',self.execute_machine(), transitions={'success':'POSTCONDITION_CHECK'})
			self.add('POSTCONDITION_CHECK',self.post_conditions("yaml/post.yaml"), transitions={'success':'ended'})

	def execute_machine(self):
		mach =  skill_sm_foo.skill_sm_foo()
		return mach

	def pre_conditions(self, yaml_filename):
		
		pre_conditions = yaml.load(open(yaml_filename).read())
		checkMachine = pre_check.PreConditionCheck(pre_conditions)
		return checkMachine
	
	def post_conditions(self, yaml_filename):
		post_conditions = yaml.load(open(yaml_filename).read())
		checkMachine = post_check.PostConditionCheck(post_conditions)
		return checkMachine

	@property    
	def inputs(self):
		return "Some Input"
    
	@property
	def outputs(self):
		return "Some Output"

	@property
	def requirements(self):
		return "Some Requirements"
