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
from actionlib import *
from actionlib.msg import *
import random

import pre_check
import post_check
import skill_state_approachpose

class SelectNavigationGoal(smach.State):
        def __init__(self):
                smach.State.__init__(self,
                        outcomes=['selected','not_selected','failed'],
                        output_keys=['base_pose'])

                self.goals = []

        def execute(self, userdata):
                # defines
                x_min = 0
                x_max = 4.0
                x_increment = 2
                y_min = -4.0
                y_max = 0.0
                y_increment = 2
                th_min = -3.14
                th_max = 3.14
                th_increment = 2*3.1414926/4

                # generate new list, if list is empty
                if len(self.goals) == 0:
                        x = x_min
                        y = y_min
                        th = th_min
                        while x <= x_max:
                                while y <= y_max:
                                        while th <= th_max:
                                                pose = []
                                                pose.append(x) # x
                                                pose.append(y) # y
                                                pose.append(th) # th
                                                self.goals.append(pose)
                                                th += th_increment
                                        y += y_increment
                                        th = th_min
                                x += x_increment
                                y = y_min
                                th = th_min

                #print self.goals
                #userdata.base_pose = self.goals.pop() # takes last element out of list
                userdata.base_pose = self.goals.pop(random.randint(0,len(self.goals)-1)) # takes random element out of list

                return 'selected'


class SkillImplementation(SkillsBase):
	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['success', 'failed', 'ended', 'reached', 'not_reached'])
		with self:
			
			self.add('PRECONDITION_CHECK', self.pre_conditions("yaml/pre.yaml"), transitions={'success':'SELECT_GOAL'})
			self.add('SELECT_GOAL',SelectNavigationGoal(),transitions={'selected':'SKILL_SM','not_selected':'failed','failed':'failed'})
			self.add('SKILL_SM',self.execute_machine(), transitions={'reached':'POSTCONDITION_CHECK', 'failed':'SELECT_GOAL', 'not_reached': 'SELECT_GOAL'})
			self.add('POSTCONDITION_CHECK',self.post_conditions("yaml/post.yaml"), transitions={'success':'ended'})

	def execute_machine(self):
		mach =  skill_state_approachpose.skill_state_approachpose()
		return mach

# base reports some diagn info 
	def pre_conditions(self, yaml_filename):
		
		pre_conditions = yaml.load(open(yaml_filename).read())
		checkMachine = pre_check.PreConditionCheck(pre_conditions)
		return checkMachine
	# tf frames comparison : base_link against map
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
