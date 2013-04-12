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

class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['selected', 'not_selected', 'failed'],
			output_keys=['base_pose'])

		self.goals = []

	def execute(self, userdata):
		# defines
		x_min = 0
		x_max = 4.0
		x_increment = 2
		y_min = -4.0
		y_max = 0.0
		y_increment = 2
		th_min = -3.14
		th_max = 3.14
		th_increment = 2 * 3.1414926 / 4

		# generate new list, if list is empty
		if len(self.goals) == 0:
			x = x_min
			y = y_min
			th = th_min
			while x <= x_max:
				while y <= y_max:
					while th <= th_max:
						pose = []
						pose.append(x) # x
						pose.append(y) # y
						pose.append(th) # th
						self.goals.append(pose)
						th += th_increment
					y += y_increment
					th = th_min
				x += x_increment
				y = y_min
				th = th_min

		#print self.goals
		#userdata.base_pose = self.goals.pop() # takes last element out of list
		userdata.base_pose = self.goals.pop(random.randint(0, len(self.goals) - 1)) # takes random element out of list

		return 'selected'

class FindTables(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['found', 'not_found', 'failed'],
			output_keys=['tables'])
		self.client = actionlib.SimpleActionClient('segmentation/trigger_segmentation', TriggerAction)

	def execute(self, userdata):
		rospy.wait_for_service('geometry_map/clear_map',10)
		try:
			clear_geom_map = rospy.ServiceProxy('geometry_map/clear_map', Trigger)
			resp1 = clear_geom_map()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		sss.move("torso", "frontleft")
		#start mapping
		goal = TriggerGoal()
		goal.start = True
		if not self.client.wait_for_server():#rospy.Duration.from_sec(5.0)):
			rospy.logerr('server not available')
			return 'failed'
		self.client.send_goal(goal)
		if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
			return 'failed'
		sss.move("torso", "frontright")
		#stop mapping
		goal = TriggerGoal()
		goal.start = False
		if not self.client.wait_for_server():#rospy.Duration.from_sec(5.0)):
			rospy.logerr('server not available')
			return 'failed'
		self.client.send_goal(goal)
		if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
			return 'failed'
		#trigger table extraction
		rospy.wait_for_service('table_extraction/get_tables',10)
		try:
			user_data.tables = rospy.ServiceProxy('table_extraction/get_tables', GetTables)
			tables = extract_tables()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


class DetectTables(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['finished', 'failed'])
		with self:

			smach.StateMachine.add('SELECT_GOAL', SelectNavigationGoal(),
																transitions={'selected':'MOVE_BASE',
																						'not_selected':'finished',
																						'failed':'failed'})

			smach.StateMachine.add('MOVE_BASE', ApproachPose(),
																	 transitions={'reached':'FIND',
																								'not_reached':'SELECT_GOAL',
																								'failed':'failed'})

			smach.StateMachine.add('FIND', FindTables(),
																	 transitions={'found':'finished',
																								'not_detected':'SELECT_GOAL',
																								'failed':'failed'})


