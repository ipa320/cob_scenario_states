import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['selected','not_selected','failed'],
			output_keys=['base_pose'])
	def execute(self, userdata):
		sss.say(["I am selecting a navigation goal"])
		sss.sleep(2)
		userdata.base_pose = "home"
		return 'selected'
