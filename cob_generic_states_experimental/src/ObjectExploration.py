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
from InvestigateTableObjects import *

class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['selected', 'not_selected', 'failed'],
			input_keys=['base_pose'],
			output_keys=['base_pose'])

		self.goals = [[0,0,-math.pi/4],[2.5,0,-3*math.pi/4]]
		self.ctr = 0

	def execute(self, userdata):

		userdata.base_pose = self.goals[self.ctr]
		self.ctr+=1
		if self.ctr == len(self.goals):
			print "All poses tried, aborting"
			return 'not_selected'
		print ("Selected ", userdata.base_pose, " as nav goal")
		return 'selected'
	
class ApproachNextTable(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['found', 'not_found', 'failed'],
			input_keys=['tables', 'polygon', 'new_computation_flag'],
			output_keys=['polygon', 'new_computation_flag'])
		return
	
	def execute(self, userdata):
		if len(userdata.tables.shapes) == 0: 
			return 'not_found'
		userdata.polygon = userdata.tables.shapes.pop()
		userdata.new_computation_flag = True
		return 'found'


class ObjectExploration(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'])
		with self:

			smach.StateMachine.add('SELECT_EXPLORATION_GOAL', SelectNavigationGoal(),
										transitions={'selected':'MOVE_BASE_EXPLORATION',
													'not_selected':'finished',
													'failed':'failed'})

			smach.StateMachine.add('MOVE_BASE_EXPLORATION', ApproachPose(),
										transitions={'reached':'DETECT_TABLES',
													'not_reached':'SELECT_EXPLORATION_GOAL',
													'failed':'failed'})

			smach.StateMachine.add('DETECT_TABLES', DetectTables(),
										transitions={'found':'APPROACH_NEXT_TABLE',
													'not_found':'SELECT_EXPLORATION_GOAL',
													'failed':'failed'})
			
			smach.StateMachine.add('APPROACH_NEXT_TABLE', ApproachNextTable(),
										transitions={'found':'APPROACH_POLYGON',
													'not_found':'SELECT_EXPLORATION_GOAL',
													'failed':'failed'})
			
			smach.StateMachine.add('APPROACH_POLYGON', ApproachPolygon(),
										transitions={'reached':'INVESTIGATE_TABLE_OBJECTS',
													'not_reached':'APPROACH_NEXT_TABLE',
													'failed':'failed'})

			smach.StateMachine.add('INVESTIGATE_TABLE_OBJECTS', InvestigateTableObjects(),
										transitions={'finished_table':'APPROACH_NEXT_TABLE',
													'approach_next_pose':'APPROACH_POLYGON',
													'failed':'failed'})

if __name__ == '__main__':
	rospy.init_node("object_exploration")
	SM = ObjectExploration()
	SM.execute()
	rospy.spin()
