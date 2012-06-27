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

from DetectObjects import *

## Detect state
#
# This state will try to detect an object in the front of care-o-bot.

class DetectObjectBackside(smach.State):
	def __init__(self,namespace, object_name = "", detector_srv = '/object_detection/detect_object',max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded','no_object','no_more_retries','failed'],
			input_keys=['object_name'],
			output_keys=['object'])

		self.object_list = DetectionArray() # UHR: DO we need that? It is not referenced 
		self.max_retries = max_retries
		self.retries = 0
		self.namespace = namespace

		self.object_detector = ObjectDetector(object_name, detector_srv,2)
	
		#ToDo: Read from yaml	
		
		self.torso_poses = []

		
		if rospy.has_param(namespace):
			params = rospy.get_param(namespace)
			torso_poses = params("torso_poses")
			
			rospy.loginfo("Found %d torso poses for state %s on ROS parameter server" %len(torso_poses) %namespace, )
			for pose in torso_poses:
				if (pose[0] == 'joints'): #(joints,0;0;0)
					self.torso_poses.append((pose[1],pose[2],pose[3]))
				elif (pose[0] == 'xyz'): #(xyz;0;5;3)
					#TODO call look at point in world (MDL)
					print "Calling LookAtPointInWorld"
				else: #string:
					self.torso_poses.append(pose)
		else:
			rospy.loginfo("No torso_poses found for state %s on ROS parameter server, taking 'home' as default" %namespace)
			self.torso_poses.append("home") # default pose

		#self.torso_poses.append("back_right")
		#self.torso_poses.append("back_right_extreme")
		#self.torso_poses.append("back")
		#self.torso_poses.append("back_extreme")
		#self.torso_poses.append("back_left")
		#self.torso_poses.append("back_left_extreme")

	def execute(self, userdata):

		sss.set_light('blue')
	
		# check if maximum retries reached
		if self.retries > self.max_retries:
			sss.set_light('yellow')
			self.retries = 0
			handle_torso = sss.move("torso","home",False)
			handle_torso.wait()
			handle_arm = sss.move("arm","look_at_table-to-folded")
			sss.set_light('blue')
			return 'no_more_retries'
		
		# move sdh as feedback
		sss.move("sdh","cylclosed",False) # UHR: could we choose a different signal? This looks a bit awkward to visitors. We have the speech output anyway - so I would just omit this statement...
		_experimental
		# make the robot ready to inspect the scene
		if type(userdata.object_name) is str:
			object_name = userdata.object_name
		else:
			object_name = "given object"

		if self.retries == 0: # only move arm, sdh and head for the first try
			sss.set_light('yellow')
			sss.say(["I will now search for the " + object_name + "."],False)
			handle_arm = sss.move("arm","folded-to-look_at_table",False)
			handle_torso = sss.move("torso","shake",False)
			handle_head = sss.move("head","back",False)
			handle_arm.wait()
			handle_head.wait()
			handle_torso.wait()
			sss.set_light('blue')

		# have an other viewing point for each retry
		handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) 
		
		# move sdh as feedback
		sss.move("sdh","home",False) # UHR: see above
		
		# wait for image to become stable
		sss.sleep(2)
	
		result = self.object_detector.execute(userdata)
		if result == 'failed':
			self.retries = 0
			sss.set_light('red')
		elif results == "no_object":
			self.retries += 1
		else: #suceeded
			self.retries = 0
			sss.move("torso","home")
		
		return result










class SM(smach.StateMachine):
        def __init__(self):
                smach.StateMachine.__init__(self,outcomes=['ended'])
                with self:
                        smach.StateMachine.add('DETECT_OBJECT_TABLE',DetectObjectBackside("detect_object_table"),
                                transitions={'no_object':'DETECT_OBJECT_TABLE',
                                        'failed':'ended',
					'succeeded':'ended',
				        'no_more_retries':'ended'})



if __name__=='__main__':
        rospy.init_node('detect_object_backside')
        sm = SM()
        sm.userdata.object_name = 'milk_box'
        sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
        sis.start()
        outcome = sm.execute()
        rospy.spin()


