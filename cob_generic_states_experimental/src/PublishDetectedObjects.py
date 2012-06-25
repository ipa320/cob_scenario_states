#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')   # todo: additional command line parameter
import rospy
import smach
import smach_ros
from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from visualization_msgs.msg import Marker


#from simple_script_server import *  # import script

class PublishDetectedObjects(smach.State):
	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['published', 'quit'],
			input_keys=['detection'])
		self.color_inc = 0
		self.vis_pub = rospy.Publisher('detected_object_marker', Marker)

	def insert_detected_object(self,detection):
		object_label = detection.label
		object_pose = detection.pose
		object_bb = detection.bounding_box_lwh

		marker = Marker()
		marker.header.frame_id = "base_link"
		marker.header.stamp = rospy.Time.now()
		marker.ns = "ExploreScene"
		marker.id = 0
		marker.type = 1
		marker.action = 0
		marker.pose = object_pose.pose
		marker.scale.x = object_bb.x
		marker.scale.y = object_bb.y
		marker.scale.z = object_bb.z
		marker.color.a = 1.0
		marker.color.r = 0.0 + self.color_inc;
		marker.color.g = 1.0 - self.color_inc;
		marker.color.b = 0.0 ;
	
		self.color_inc = self.color_inc + 0.1

		self.vis_pub.publish( marker );

	
	def execute(self,userdata):
		#publish all detected objects
		self.insert_detected_object(userdata.detection)
		return 'published'


class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('STATE',PublishDetectedObjects(),
				transitions={'quit':'STATE',
					'published':'STATE'})

	

if __name__=='__main__':
	rospy.init_node('PublishDetectedObjects')
	sm = SM()
	sm.userdata.detection = Detection()
	sm.userdata.detection.bounding_box_lwh.x = 0.1
	sm.userdata.detection.bounding_box_lwh.y = 0.1
	sm.userdata.detection.bounding_box_lwh.z = 0.1
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	#sis.stop()'quit':{'SelectObjectFromTablet':'quit'}})
            
 
#rospy.init_node('eHealth2012')
#sm = SM()
#outcome = sm.execute()
#rospy.spin()


