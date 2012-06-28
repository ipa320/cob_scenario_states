#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

from math import *
import copy

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *

from ObjectDetector import *


## Detect front state
#
# This state will try to detect an object in the back of care-o-bot.
# It encorporates the movement of the arm to ensure that is not in the vision space  of the cameras

class DetectObjectFrontside(smach.State):
	def __init__(self,namespace, object_names = [], detector_srv = '/object_detection/detect_object'):
		smach.State.__init__(
			self,
			outcomes=['detected','not_detected','failed'],
			input_keys=['object_names'],
			output_keys=['objects'])

		self.torso_poses = []
		self.object_names = object_names
		self.object_detector = ObjectDetector(object_names, detector_srv)
	
		#ToDo: Read from yaml	
		if rospy.has_param(namespace):
			params = rospy.get_param(namespace)
			torso_poses = params["torso_poses"]
			for pose in torso_poses:
				if (pose[0] == 'joints'): #(joints,0;0;0)
					self.torso_poses.append((pose[1],pose[2],pose[3]))
				elif (pose[0] == 'xyz'): #(xyz;0;5;3)
					#TODO call look at point in world (MDL)
					print "Call LookAtPointInWorld"
				else: #string:
					self.torso_poses.append(pose)
		else:
			rospy.loginfo("No torso_poses found for state %s on ROS parameter server, taking 'home' as default" %namespace)
			self.torso_poses.append("home") # default pose

	def execute(self, userdata):
		# determine object_names
		if self.object_names != []:
			object_names = self.object_names
		else
			object_names = userdata.object_names
		
		for name in object_names:
			if type(name) is str:
			
			else:# this should never happen
				rospy.logerr("Invalid userdata 'object_names':%s",object_names)
				sss.set_light('red')
				return 'failed'

		sss.set_light('blue')


		if self.retries == 0: # only move sdh and head for the first try
			sss.set_light('yellow')
			sss.say(["I will now search for the " + object_name + "."],False)
			handle_torso = sss.move("torso","shake",False)
			handle_head = sss.move("head","front",False)
			handle_head.wait()
			handle_torso.wait()
			sss.set_light('blue')

	
		objects = []
		for pose in self.torso_poses:
			# have an other viewing point for each retry
			handle_torso = sss.move("torso",pose)
			
			
			result,detected_object = self.object_detector.execute(userdata)
			if result == 'failed':
				break
			elif result == 'detected'
				objects.append(detected_object)
			
			
			# wenn von jedem eines gefunden, dann aufhoeren
			break
				


		#write to userdata

		# ... cleanup robot components
		sss.move("torso","home")
		sss.set_light('green')

		if len(userdata.objects) == 0:
			return 'not_detected'
		
		return result







class SM(smach.StateMachine):
        def __init__(self):
                smach.StateMachine.__init__(self,outcomes=['ended'])
                with self:
                        smach.StateMachine.add('DETECT_OBJECT_TABLE',DetectObjectFrontside("detect_object_table"),
                                transitions={'not_detected':'DETECT_OBJECT_TABLE',
                                        'failed':'ended',
					'detected':'ended'})



if __name__=='__main__':
        rospy.init_node('detect_object_frontside')
        sm = SM()
        sm.userdata.object_name = 'milk'
        sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
        sis.start()
        outcome = sm.execute()
        rospy.spin()

