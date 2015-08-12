#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from ApproachPose import *
from PickObject import *
from PutObjectOnTray import *
from DeliverObject import *

# fetch 'object' from 'location'
class FetchObject(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['fetched','not_fetched','failed'],
                                    input_keys=['base_pose'])
        with self:

            smach.StateMachine.add('MOVE_TO_OBJECT',ApproachPose(),
                                   transitions={'reached':'PICK',
                                                'not_reached':'not_fetched',
                                                'failed':'failed'})

            smach.StateMachine.add('PICK',PickObject(),
                                   transitions={'picked':'PUT_ON_TRAY',
                                                'not_picked':'not_fetched',
                                                'failed':'failed'})

            smach.StateMachine.add('PUT_ON_TRAY',PutObjectOnTray(),
                                   transitions={'put':'DELIVER',
                                                'not_put':'not_fetched',
                                                'failed':'failed'})

            smach.StateMachine.add('DELIVER',DeliverObject(),
                                   transitions={'delivered':'fetched',
                                                'not_delivered':'not_fetched',
                                                'failed':'failed'})