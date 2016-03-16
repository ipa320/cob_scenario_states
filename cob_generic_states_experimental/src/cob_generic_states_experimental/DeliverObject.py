#!/usr/bin/python

import rospy
import smach
import smach_ros

from cob_generic_states_experimental.ApproachPose import *
from cob_generic_states_experimental.HandOut import *

class DeliverObject(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['delivered','not_delivered','failed'])
        with self:

            smach.StateMachine.add('MOVE_BASE',ApproachPose(),
                                   transitions={'reached':'HAND_OUT',
                                                'not_reached':'not_delivered',
                                                'failed':'failed'})

            smach.StateMachine.add('HAND_OUT',HandOut(),
                                   transitions={'handed_out':'delivered',
                                                'not_handed_out':'not_delivered',
                                                'failed':'failed'})
