import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class WaitForOpenDoor(smach.State):
	def __init__(self,number_of_doors = 1):
		smach.State.__init__(self, 
			outcomes=['open','not_open','failed'],
			output_keys=['door_pose'])
		self.number_of_doors = number_of_doors
		
	def execute(self, userdata):
		sss.say(["I am waiting for an open door."])
		
		if self.number_of_doors <= 1:
			rospy.sleep(2)
			sss.say(["Door is open."])
			return 'open'
		
		side = ""
		while not (side == "left" or side == "right"):
			print "which door is open? [left,right]"
			side = sss.wait_for_input()
		
		sss.say(["Door on " + side + " side is open."])
		userdata.door_pose = "elevator_in_" + side
		return 'open'
