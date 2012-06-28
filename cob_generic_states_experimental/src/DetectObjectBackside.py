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

## DetectObjectBackside state
#
# This state will try to detect an object in the front of care-o-bot.
# \param namespace Indicates the namespace for the torso poses pushed onto the parameter server (we could have several detection states
#        with different desired torso poses)
# \param object_names Input list containing the objects that are looked for
# \param detector_srv Detection service that is called for the actual obhect detection
#    	 e.g. /object_detection/detect_object
# \param mode logical mode for detection result \	
#	 all: outcome is success only if all objects in object_names are detected
#	 one: outcome is success if at least one object from object_names list is detected 

class DetectObjectBackside(smach.State):
	def __init__(self,namespace, object_names = [], detector_srv = '/object_detection/detect_object',mode='all'):
		smach.State.__init__(
			self,
			outcomes=['detected','not_detected', 'failed'],
			input_keys=['object_names'],
			output_keys=['objects'])

		
		if mode not in ['all','one']:
			rospy.logwarn("Invalid mode: must be 'all', or 'one', selecting default value = 'all'")
			self.mode = 'all'	
		else:
			self.mode = mode

		self.object_detector = ObjectDetector(namespace, object_names, detector_srv,mode)
	

	def execute(self, userdata):

		sss.set_light('blue')

		#Preparations for object detection	
		handle_torso = sss.move("torso","home",False)
		handle_arm = sss.move("arm","folded-to-look_at_table",False)
		handle_head = sss.move("head","back",False)
		sss.set_light('yellow')
		handle_arm.wait()
		handle_torso.wait()
		handle_head.wait()
		sss.set_light('blue')

	
		result, userdata.objects = self.object_detector.execute(userdata)

		handle_arm = sss.move("arm","look_at_table-to-folded")
		sss.set_light('yellow')
		handle_arm.wait()
		sss.set_light('green')
		
		return result










class SM(smach.StateMachine):
        def __init__(self):
                smach.StateMachine.__init__(self,outcomes=['ended'])
                with self:
                        smach.StateMachine.add('DETECT_OBJECT_TABLE',DetectObjectBackside("detect_object_table"),
                                transitions={'not_detected':'ended',
                                        'failed':'ended',
					'detected':'ended',
				        })



if __name__=='__main__':
        rospy.init_node('detect_object_backside')
        sm = SM()
        sm.userdata.object_names = ['milk_box']
        sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
        sis.start()
        outcome = sm.execute()
        rospy.spin()


