#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from DetectObjectsBackside import *

class SetObjectName(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], 
							input_keys=['object_names'],
							output_keys=['object_names'])
	
	def execute(self, userdata):
		userdata.object_names = ['all']
		return 'success'


class InvestigateTableObjects(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished_table', 'approach_next_pose', 'failed'])
		with self:

			smach.StateMachine.add('SET_STUPID_OBJECT_NAME', SetObjectName(),
										transitions={'success':'DETECT_OBJECTS'})
			smach.StateMachine.add('DETECT_OBJECTS', DetectObjectsBackside(),
										transitions={'detected':'approach_next_pose',
													'not_detected':'approach_next_pose',
													'failed':'failed'})

if __name__ == '__main__':
	rospy.init_node("object_exploration")
	SM = InvestigateTables()
	SM.execute()
	rospy.spin()
