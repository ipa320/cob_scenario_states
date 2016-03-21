#!/usr/bin/python

import rospy
import smach
import smach_ros

import random
from nav_msgs.srv import *
from cob_object_detection_msgs.msg import *

from cob_generic_states_experimental.ApproachPose import *
from cob_generic_states_experimental.DetectObjectsFrontside import *
from cob_generic_states_experimental.MoveYourself import *
from cob_generic_states_experimental.Explore import *

class ExploreAndMoveYourself(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('SELECT_GOAL',SelectNavigationGoal(),
                                   transitions={'selected':'MOVE_BASE',
                                                'not_selected':'finished',
                                                'failed':'failed'})
            
            smach.StateMachine.add('MOVE_BASE',ApproachPose(),
                                   transitions={'reached':'DETECT',
                                                'not_reached':'SELECT_GOAL',
                                                'failed':'failed'})

            smach.StateMachine.add('DETECT',DetectObjectsFrontside(['milk','pringles'],mode="one"),
                                   transitions={'detected':'ANNOUNCE',
                                                'not_detected':'ANNOUNCE',
                                                'failed':'failed'})

            smach.StateMachine.add('ANNOUNCE',AnnounceFoundObjects(),
                                   transitions={'announced':'MOVE_YOURSELF',
                                                'not_announced':'MOVE_YOURSELF',
                                                'failed':'failed'})

            smach.StateMachine.add('MOVE_YOURSELF',MoveYourself(),
                                   transitions={'succeeded':'SELECT_GOAL',
                                                'failed':'failed'})
















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',ExploreAndMoveYourself(),
            		transitions={'finished':'ended',
            					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('ExploreAndMoveYourself')
	sm = SM()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
