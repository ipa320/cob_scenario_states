#!/usr/bin/python

import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from std_srvs.srv import Trigger

class HandOut(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['handed_out','not_handed_out','failed'],
			input_keys=['object'])

	def execute(self, userdata):
		sss.say(["Here is your " + userdata.object.label + ". Please help yourself."],False)
		sss.move("torso","nod",False)
		
		try:
			rospy.wait_for_service('occupied',10)
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			return 'failed'

		wait_time = 0
		while True:
			if wait_time > 10:
				return 'not_handed_out'
			try:
				tray_service = rospy.ServiceProxy('occupied', Trigger)			
				req = TriggerRequest()
				res = tray_service(req)
				print "waiting for tray to be not occupied any more"
				if(res.success.data == False):
					break
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
				return 'failed'
			sleep_time = 0.2
			wait_time += sleep_time
			rospy.sleep(sleep_time)
		return 'handed_out'
