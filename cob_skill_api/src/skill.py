#!/usr/bin/python
import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib.msg import *

import yaml

#from preconditionCheckLib import *

class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                TestAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()
        elif msg.goal == 2:
            self._sas.set_preempted()


class PreConditionCheck(smach.State):

	def __init__(self, yaml_filename):
		smach.State.__init__(self, outcomes=['success','failed'])
		self.activated_checks = yaml.load(open(yaml_filename).read())

	def execute(self, userdata):
		for check in self.activated_checks:
			if not (lambda check: check == "success"):    
				return 'failed'
			return 'success'
      
class PostConditionCheck(smach.State):

	def __init__(self, yaml_filename):
		smach.State.__init__(self, outcomes=['success','failed'])

		self.activated_checks = yaml.load(open(yaml_filename).read())

	def execute(self, userdata):
		for check in self.activated_checks:
                        if not (lambda check: check == "success"):
                                return 'failed'
                        return 'success'

#above is generic
#===================

class skill_sm_foo(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['success','failed'])
		with self:
			smach.StateMachine.add('BAR1', smach_ros.SimpleActionState('BAR', TestAction),  transitions={'succeeded':'success', 'aborted':'success', 'preempted':'success'})

class Skill_Foo(smach.StateMachine):

	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['success', 'failed', 'ended'])
		with self:
			smach.StateMachine.add('PRECONDITION_CHECK', PreConditionCheck("pre.yaml"), transitions={'success':'POSTCONDITION_CHECK'})
			smach.StateMachine.add('SKILL_SM', skill_sm_foo(), transitions={'success':'POSTCONDITION_CHECK'})
			smach.StateMachine.add('POSTCONDITION_CHECK',PostConditionCheck("post.yaml"), transitions={'success':'ended'})
            


if __name__ == "__main__":

	rospy.init_node('skill_template')
	server = TestServer('BAR')
	sm = Skill_Foo()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
