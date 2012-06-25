#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from ApproachPose import *
from SelectNavigationGoal import *
from DetectObjects import *
from PublishDetectedObjects import *

class Explore(smach.StateMachine):
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


            smach.StateMachine.add('DETECT',DetectObjectsFrontside(),
                                   transitions={'detected':'SELECT_GOAL',
                                                'not_detected':'SELECT_GOAL',
                                                'failed':'failed'})

 #           smach.StateMachine.add('PUBLISH_MARKER',PublishDetectedObjects(),
 #                                  transitions={'published':'SELECT_GOAL'})


















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Explore(),
            		transitions={'finished':'ended',
            					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('Explore')
	sm = SM()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
