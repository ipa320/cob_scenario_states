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
        smach.StateMachine.__init__(self,outcomes=['pressed','not_pressed','failed'])
        with self:

            smach.StateMachine.add('DETECT',DetectObjectsBackside('milk'),
                                   transitions={'found':'PRESS',
                                                'not_found':'not_pressed',
                                                'failed':'failed'})

            smach.StateMachine.add('PRESS',PressButton(),
                                   transitions={'pressed':'pressed',
                                                'not_pressed':'not_pressed',
                                                'failed':'failed'})