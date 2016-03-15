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



class DetectTables(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['found', 'not_found', 'failed'],
			input_keys=['tables'],
			output_keys=['tables'])
		self.client = actionlib.SimpleActionClient('/trigger_segmentation', TriggerAction)

	def execute(self, userdata):
		#stop mapping
#		goal = TriggerGoal()
#		goal.start = False
#		if not self.client.wait_for_server(rospy.Duration.from_sec(3.0)):#rospy.Duration.from_sec(5.0)):
#			rospy.logerr('Trigger action server not available')
#			return 'failed'

		rospy.wait_for_service('geometry_map/clear_map',2.0)
		try:
			clear_geom_map = rospy.ServiceProxy('geometry_map/clear_map', Trigger)
			resp1 = clear_geom_map()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			
		sss.move("head","front")
		sss.move("torso", "front_left")
		
		#start mapping
#		goal.start = True
#		if not self.client.wait_for_server(rospy.Duration.from_sec(1.0)):
#			rospy.logerr('Trigger action server not available')
#			return 'failed'
#		self.client.send_goal(goal)
#		if not self.client.wait_for_result():
#			return 'failed'

		sss.move("torso", "front_right")
		
		#stop mapping
#		goal = TriggerGoal()
#		goal.start = False
#		if not self.client.wait_for_server(rospy.Duration.from_sec(1.0)):#rospy.Duration.from_sec(5.0)):
#			rospy.logerr('Trigger action server not available')
#			return 'failed'
#		self.client.send_goal(goal)
#		if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
#			return 'failed'

		#trigger table extraction
		rospy.wait_for_service('table_extraction/get_tables',3.0)
		try:
			extract_tables = rospy.ServiceProxy('table_extraction/get_tables', GetTables)
			userdata.tables = extract_tables().tables
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		print "Found %d tables"%len(userdata.tables.shapes)
		if len(userdata.tables.shapes) == 0:
			return 'not_found'
		return 'found'


#class DetectTables(smach.StateMachine):
#	def __init__(self):
#		smach.StateMachine.__init__(self,
#			outcomes=['finished', 'failed'])
#		with self:
#
#			smach.StateMachine.add('SELECT_GOAL', SelectNavigationGoal(),
#										transitions={'selected':'MOVE_BASE',
#													'not_selected':'finished',
#													'failed':'failed'})
#
#			smach.StateMachine.add('MOVE_BASE', ApproachPose(),
#										transitions={'reached':'FIND',
#													'not_reached':'SELECT_GOAL',
#													'failed':'failed'})
#
#			smach.StateMachine.add('FIND', FindTables(),
#										transitions={'found':'finished',
#													'not_found':'SELECT_GOAL',
#													'failed':'failed'})
#
#if __name__ == '__main__':
#	rospy.init_node("detect_tables")
#	SM = DetectTables()
#	SM.execute()
#	rospy.spin()
