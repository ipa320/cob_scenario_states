#!/usr/bin/python

import rospy
import smach
import smach_ros

from cob_generic_states_experimental.ApproachPose import *
from cob_generic_states_experimental.DetectAndPressButton import *
from cob_generic_states_experimental.WaitForOpenDoor import *


class ChangeFloor(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['changed','not_changed','failed'])
        with self:

            smach.StateMachine.add('MOVE_TO_ELEVATOR',ApproachPose('elevator_button'),
                                   transitions={'reached':'CALL_ELEVATOR',
                                                'not_reached':'not_changed',
                                                'failed':'failed'})

            smach.StateMachine.add('CALL_ELEVATOR',DetectAndPressButton(),
                                   transitions={'pressed':'MOVE_BACK',
                                                'not_pressed':'not_changed',
                                                'failed':'failed'})

            smach.StateMachine.add('MOVE_BACK',ApproachPose('elevator_out'),
                                   transitions={'reached':'WAIT_FOR_OPEN_DOOR',
                                                'not_reached':'not_changed',
                                                'failed':'failed'})

            smach.StateMachine.add('WAIT_FOR_OPEN_DOOR',WaitForOpenDoor(2),
                                   transitions={'open':'MOVE_INTO_ELEVATOR',
                                                'not_open':'not_changed',
                                                'failed':'failed'})
            
            # TODO: define pose for left and right pose in elevator
            smach.StateMachine.add('MOVE_INTO_ELEVATOR',ApproachPose(),
                                   transitions={'reached':'SELECT_FLOOR',
                                    'not_reached':'not_changed',
                                    'failed':'failed'},
                                   remapping={'base_pose':'door_pose'})
            
            smach.StateMachine.add('SELECT_FLOOR',DetectAndPressButton(),
                       transitions={'pressed':'WAIT_FOR_LEAVING',
                                    'not_pressed':'not_changed',
                                    'failed':'failed'})

            smach.StateMachine.add('WAIT_FOR_LEAVING',WaitForOpenDoor(),
                                   transitions={'open':'MOVE_OUT_OF_ELEVATOR',
                                                'not_open':'not_changed',
                                                'failed':'failed'})
            
            smach.StateMachine.add('MOVE_OUT_OF_ELEVATOR',ApproachPose('elevator_out'),
                       transitions={'reached':'changed',
                                    'not_reached':'not_changed',
                                    'failed':'failed'})
