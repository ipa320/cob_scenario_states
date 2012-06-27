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


## detect_obect
#
# In this class the actual object detection is executed exactly one time
# No further actions are executed
class ObjectDetector:
	def __init__(self, object_names = [], detector_srv = '/object_detection/detect_object', min_dist = 2  ):
		self.detector_srv = detector_srv 
		self.object_name = object_name
		self.min_dist = min_dist

	def execute(self, userdata):
		# determine object name
		if self.object_name != "":
			object_name = self.object_name
		elif type(userdata.object_name) is str:
			object_name = userdata.object_name
		else: # this should never happen
			rospy.logerr("Invalid userdata 'object_name'")
			return 'failed'

		# check if object detection service is available
		print self.detector_srv
		try:
			rospy.wait_for_service(self.detector_srv,10)
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			return 'failed'

		# call object detection service
		try:
			detector_service = rospy.ServiceProxy(self.detector_srv, DetectObjects)
			req = DetectObjectsRequest()
			req.object_name.data = object_name
			res = detector_service(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
			
		# check for no objects
		if len(res.object_list.detections) <= 0:
			rospy.logerr("No objects found")
			return 'not_detected'
		
		# check if an object could be found within the min_dist start value
		if obj.label == "":
			rospy.logerr("Object not within target range")
			return 'not_detected'

		#check if label of object fits to requested object_name
		if obj.label != object_name:
			rospy.logerr("The object name doesn't fit.")
			return 'not_detected'

		# HACK for timestamp
		obj.pose.header.stamp = rospy.Time.now()

		# we succeeded to detect an object
		userdata.object = obj
		
		return 'detected'
		
