import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class HandOut(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['handed_out','not_handed_out','failed'],
			input_keys=['object_name'])
	def execute(self, userdata):
		sss.say(["I am handing " + userdata.object_name + " out now."])
		sss.sleep(2)
		return 'handed_out'
