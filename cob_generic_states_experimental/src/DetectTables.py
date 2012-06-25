import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class DetectTables(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['found','not_found','failed'],
			input_keys=[])
	def execute(self, userdata):
		sss.say(["I am detecting tables now."])
		sss.sleep(2)
		sss.say(["I found one table."])
		return 'found'
