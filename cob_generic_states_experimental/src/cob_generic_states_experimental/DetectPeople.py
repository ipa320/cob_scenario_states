#!/usr/bin/python

import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class DetectPeople(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['found','not_found','failed'],
			input_keys=[])
	def execute(self, userdata):
		sss.say("sound", ["I am detecting people now."])
		sss.sleep(2)
		sss.say("sound", ["I found two persons."])
		return 'found'
