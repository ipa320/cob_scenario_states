#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from DetectObjects import *
from Grasp import *

class PickObject(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['picked','not_picked','failed'])
        with self:

            smach.StateMachine.add('DETECT',DetectObjectsBackside(),
                                   transitions={'detected':'GRASP',
                                                'not_detected':'not_picked',
                                                'failed':'failed'})

            smach.StateMachine.add('GRASP',Grasp(),
                                   transitions={'grasped':'picked',
                                                'not_grasped':'DETECT',
                                                'failed':'failed'})