#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from cob_3d_mapping_msgs.msg import *
from cob_3d_mapping_msgs.srv import *
from cob_srvs.srv import Trigger

from ApproachPose import *
from ApproachPolygon import *
from DetectTables import *

class SelectTable(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['selected', 'not_selected', 'failed'],
			input_keys=['tables'],
			output_keys=['polygon'])

		self.ctr = 0

	def execute(self, userdata):
		if self.ctr == len(self.goals):
			print "All tables tried, aborting"
			return 'not_selected'
		userdata.polygon = userdata.tables[self.ctr]
		self.ctr+=1
		return 'selected'


class InvestigateTables(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'])
		with self:

			smach.StateMachine.add('SELECT_TABLE', SelectTable(),
										transitions={'selected':'SELECT_TABLE_APPROACH_GOAL',
													'not_selected':'finished',
													'failed':'failed'})
			
			smach.StateMachine.add('SELECT_TABLE_APPROACH_GOAL', ComputeNavigationGoal(),
										transitions={'computed':'MOVE_BASE_TABLE_APPROACH',
													'not_computed':'SELECT_TABLE',
													'failed':'failed'})

			smach.StateMachine.add('MOVE_BASE_TABLE_APPROACH', ApproachPose(),
											transitions={'reached':'finished',
											'not_reached':'SELECT_TABLE_APPROACH_GOAL',
											'failed':'failed'})

if __name__ == '__main__':
	rospy.init_node("object_exploration")
	SM = InvestigateTables()
	SM.execute()
	rospy.spin()
