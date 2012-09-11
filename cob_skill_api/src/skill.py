#!/usr/bin/python
import roslib
roslib.load_manifest('cob_skill')
import rospy
import smach
import smach_ros

from preconditionCheckLib import *

class PreConditionCheck(smach.State):
    def __init__(self, yaml_filename):
        smach.State.__init__(self, outcomes=['success','failed'])
        activated_checks = read_yaml_file(yaml_file)
    def execute(self, userdata):
        for(check in activated_checks):
            if(not lamda check()):    
                return 'failed'
        return 'success'
      
class PostConditionCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])

    def execute(self, userdata):
        pass

#above is generic
===================

class skill_sm_foo(smach.StateMachine):
    smach.StateMachine.__init__(self, outcomes=['success','failed'])
    with self:
        smach.StateMachine.add('BAR1', SimpleActionStateBar(), transitions={'success':'success'})
        #ODER
        smach.StateMachine.add('BAR1', Skill_Bar(), transitions={'success':'success'})
        

class Skill_Foo(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['ended'])
        with self:
            smach.StateMachine.add('PRECONDITION_CHECK', PreConditionCheck("path_to_yaml_file"), transitions={'success':'SKILL_SM'})
            smach.StateMachine.add('SKILL_SM', skill_sm_foo, transitions={'success':'POSTCONDITION_CHECK'}
            smach.StateMachine.add('POSTCONDITION_CHECK', PostConditionCheck("path_to_yaml_file"), transitions={'success':'ended'})
            


if __name__=='__main__':
    rospy.init_node('skill_template')
    sm = Skill()
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
