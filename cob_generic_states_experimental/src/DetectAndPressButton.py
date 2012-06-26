#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from DetectObjects import *
from PressButton import *

class DetectAndPressButton(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['pressed','not_pressed','failed'],
                                         input_keys=['object_name'])
        with self:

            smach.StateMachine.add('DETECT',DetectObjectsBackside('Care-O-bot'),
                                   transitions={'detected':'PRESS',
                                                'not_detected':'not_pressed',
                                                'failed':'failed'},
                                   remapping={'object':'button'})

            smach.StateMachine.add('PRESS',PressButton(),
                                   transitions={'pressed':'pressed',
                                                'not_pressed':'not_pressed',
                                                'failed':'failed'})


























class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',DetectAndPressButton(),
                    transitions={'pressed':'ended',
                                'not_pressed':'ended',
                                'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('DetectAndPressButton')
    sm = SM()
    sm.userdata.object_name
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
