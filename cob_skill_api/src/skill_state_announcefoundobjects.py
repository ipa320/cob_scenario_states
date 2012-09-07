import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()	

from abc_state_skill import SkillsState

class skill_state_announcefoundobjects(SkillsState):
	def __init__(self):
		rospy.loginfo("Initializing Announce objects skill")

		smach.State.__init__(self, 
			outcomes=['announced','not_announced','failed'],
			input_keys=['objects'],
			output_keys=['objects'])
			
	def execute(self, userdata):
		rospy.loginfo("Executing Announce Objects Skill")
		object_names = ""
		for obj in userdata.objects:
			object_names += obj.label + ", "
		
		if object_names != "":
			sss.say(["I found: " + object_names])
		else:
			sss.say(["I found: nothing"])
		userdata.objects = []
		return 'announced'
