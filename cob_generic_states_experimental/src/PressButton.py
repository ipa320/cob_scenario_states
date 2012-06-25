import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class PressButton(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['pressed','not_pressed','failed'],
			input_keys=[])
	def execute(self, userdata):
		sss.say(["I am pressing a button now."])
		sss.move("arm",["hold","pregrasp","hold","folded"])
		sss.say(["I pressed the button."])
		return 'pressed'
