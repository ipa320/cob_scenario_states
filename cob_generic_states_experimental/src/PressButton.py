#!/usr/bin/python

import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

import tf
from sensor_msgs.msg import *
from cob_object_detection_msgs.msg import *

class PressButton(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['pressed','not_pressed','failed'],
			input_keys=['button'])
		self.listener = tf.TransformListener()
			
	def execute(self, userdata):
		sss.say(["I am pressing a button now."])
		#print userdata.button
		
		button_pose_bl = self.listener.transformPose("/base_link", userdata.button.pose)
		[r,p,y] = tf.transformations.euler_from_quaternion([userdata.button.pose.pose.orientation.x,userdata.button.pose.pose.orientation.y,userdata.button.pose.pose.orientation.z,userdata.button.pose.pose.orientation.w])
		
		
		# calculate ik solutions for pre grasp configuration
		pre_button_js, error_code = sss.calculate_ik(["base_link",[-1.0 , 0.1, 1.0],[r, p, y]])		
		if(error_code.val != error_code.SUCCESS):
			if error_code.val != error_code.NO_IK_SOLUTION:
				sss.set_light('red')
			rospy.logerr("Ik pre_button Failed")
			return 'not_pressed'
		
		button_js, error_code = sss.calculate_ik(["arm_7_link",[0.0, 0.0, 0.1],[0.0, 0.0, 0.0]])		
		if(error_code.val != error_code.SUCCESS):
			if error_code.val != error_code.NO_IK_SOLUTION:
				sss.set_light('red')
			rospy.logerr("Ik button Failed")
			return 'not_pressed'
		
		handle_arm = sss.move("arm", [list(pre_button_js.position)])
		rospy.sleep(10)
		handle_arm = sss.move("arm", [list(button_js.position)])


#		test_js, error_code = sss.calculate_ik(["arm_7_link",[0.0, 0.0, 0.1],[0.0, 0.0, 0.0]])		
#		if(error_code.val != error_code.SUCCESS):
#			if error_code.val != error_code.NO_IK_SOLUTION:
#				sss.set_light('red')
#			rospy.logerr("Ik pre_button Failed")
#			return 'not_pressed'
#		handle_arm = sss.move("arm", [list(test_js.position)])
		
		return 'pressed'



















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',PressButton(),
            		transitions={'pressed':'ended',
            					'not_pressed':'ended',
            					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('PressButton')
	sm = SM()
	sm.userdata.button = Detection()
	sm.userdata.button.pose.header.stamp = rospy.Time.now()
	sm.userdata.button.pose.header.frame_id = "base_link"
	sm.userdata.button.pose.pose.position.x = -0.5
	sm.userdata.button.pose.pose.position.y = -0.1
	sm.userdata.button.pose.pose.position.z = 1.0
	[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(0, -1.5708, 1.5708) # rpy 
	sm.userdata.button.pose.pose.orientation.x = new_x
	sm.userdata.button.pose.pose.orientation.y = new_y
	sm.userdata.button.pose.pose.orientation.z = new_z
	sm.userdata.button.pose.pose.orientation.w = new_w

	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
