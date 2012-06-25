#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from ApproachPose import *
from SelectNavigationGoal import *

class WonderAround(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
								outcomes=['finished','failed'],
								output_keys=['base_pose'])
        with self:

            smach.StateMachine.add('SELECT_GOAL',SelectNavigationGoal(),
                                   transitions={'selected':'MOVE_BASE',
                                                'not_selected':'finished',
                                                'failed':'failed'})
            
            smach.StateMachine.add('MOVE_BASE',ApproachPose(),
                                   transitions={'reached':'SELECT_GOAL',
                                                'not_reached':'SELECT_GOAL',
                                                'failed':'failed'})