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

class DetectObjectsFrontside(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['detected','not_detected','failed'],
			input_keys=[])
	def execute(self, userdata):
		sss.say(["I am detecting objects now."])
		return 'detected'

class DetectObjectsBackside(smach.State):
	def __init__(self,object_name = "", detector = "/object_detection/detect_object" ,max_retries = 6):
		smach.State.__init__(
			self,
			outcomes=['detected','not_detected','failed'],
			input_keys=['object_name'],
			output_keys=['object'])

		self.object_list = DetectionArray()
		self.max_retries = max_retries
		self.object_name = object_name
		self.srv_name_object_detection = detector
		
		self.torso_poses = []
		self.torso_poses.append("back_right")
		self.torso_poses.append("back_right_extreme")
		self.torso_poses.append("back")
		self.torso_poses.append("back_extreme")
		self.torso_poses.append("back_left")
		self.torso_poses.append("back_left_extreme")

	def execute(self, userdata):
		# determine object name
		if self.object_name != "":
			object_name = self.object_name
		elif type(userdata.object_name) is str:
			object_name = userdata.object_name
		else: # this should never happen
			rospy.logerr("Invalid userdata 'object_name'")
			self.retries = 0
			sss.set_light('red')
			return 'failed'

		# make the robot ready to inspect the scene
		sss.set_light('yellow')
		sss.say(["I will now search for the " + object_name + "."],False)
		handle_arm = sss.move("arm","folded-to-look_at_table",False)
		handle_torso = sss.move("torso","shake",False)
		handle_head = sss.move("head","back",False)
		handle_arm.wait()
		handle_head.wait()
		handle_torso.wait()
		sss.set_light('blue')
		
		retries = 0
		while retries < self.max_retries:	
			handle_torso = sss.move("torso",self.torso_poses[retries % len(self.torso_poses)]) # have an other viewing point for each retry

			# check if object detection service is available
			try:
				rospy.wait_for_service(self.srv_name_object_detection,10)
			except rospy.ROSException, e:
				print "Service not available: %s"%e
				sss.set_light('red')
				return 'failed'
	
			# call object detection service
			try:
				detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
				req = DetectObjectsRequest()
				req.object_name.data = object_name
				res = detector_service(req)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
				sss.set_light('red')
				return 'failed'
				
			# check for no objects
			if len(res.object_list.detections) <= 0:
				rospy.logerr("No objects found")
				retries += 1
				continue
			
			# select nearest object in x-y-plane in head_camera_left_link
			min_dist = 2 # start value in m
			obj = Detection()
			for item in res.object_list.detections:
				dist = sqrt(item.pose.pose.position.x*item.pose.pose.position.x+item.pose.pose.position.y*item.pose.pose.position.y)
				if dist < min_dist:
					min_dist = dist
					obj = copy.deepcopy(item)
			
			# check if an object could be found within the min_dist start value
			if obj.label == "":
				rospy.logerr("Object not within target range")
				retries += 1
				continue
	
			#check if label of object fits to requested object_name
			if obj.label != object_name:
				rospy.logerr("The object name doesn't fit.")
				retries += 1
				continue
	
			# we succeeded to detect an object
			userdata.object = obj
			sss.move("torso","home")
			return 'detected'
		# end while
		
		# no object found and maximum retries reached --> move back all components to default position
		sss.set_light('yellow')
		handle_torso = sss.move("torso","home",False)
		handle_torso.wait()
		handle_arm = sss.move("arm","look_at_table-to-folded")
		sss.set_light('blue')
		return 'not_detected'
			
