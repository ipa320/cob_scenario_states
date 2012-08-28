import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import random
from nav_msgs.msg import Odometry

from simple_script_server import *
sss = simple_script_server()	

from abc_sm_skill import SkillsSM

class skill_sm_approachpose(SkillsSM):

	def __init__(self, components = [], pose = "", mode = "omni", timeout = 30.0):
			smach.State.__init__(
				self,
				outcomes=['reached', 'not_reached', 'failed'],
				input_keys=['base_pose'])

			self.components = components
			
			# Subscriber to base_odometry
			rospy.Subscriber("/base_controller/odometry", Odometry, self.callback)
			self.pose = pose
			self.mode = mode
			self.is_moving = False
			self.warnings = ["I can not reach my target position because my path or target is blocked.","My path is blocked.", "I can not reach my target position."]
			self.timeout = timeout

	#Callback for the /base_controller/odometry subscriber
	def callback(self,msg):
		r = 0.01 # error range in m/s or rad/s
		if (abs(msg.twist.twist.linear.x) > r) or (abs(msg.twist.twist.linear.y) > r) or (abs(msg.twist.twist.angular.z) > r): 
			self.is_moving = True
		else:
			self.is_moving = False
		return 

	def execute(self, userdata):

		# determine target position
		if self.pose != "":
			pose = self.pose
		elif type(userdata.base_pose) is str:
			pose = userdata.base_pose
		elif type(userdata.base_pose) is list:
			pose = []
			pose.append(userdata.base_pose[0])
			pose.append(userdata.base_pose[1])
			pose.append(userdata.base_pose[2])
		else: # this should never happen
			rospy.logerr("Invalid userdata 'pose'")
			return 'failed'

		# try reaching pose
		handle_base = sss.move("base", pose, mode=self.mode, blocking=False)

		# init variables
		stopping_time = 0.0
		announce_time = 0.0
		freq = 2.0 # Hz
		yellow = False
	
		# check for goal status
		while not rospy.is_shutdown():
		
			# finished with succeeded
			if (handle_base.get_state() == 3):
				if ("light" in self.components):
					sss.set_light('green')
				return 'reached'
			# finished with aborted
			elif (handle_base.get_state() == 4):
				if ("light" in self.components):
					sss.set_light('green')
				return 'not_reached'
			# finished with preempted or canceled
			elif (handle_base.get_state() == 2) or (handle_base.get_state() == 8):
				if ("light" in self.components):
					sss.set_light('green')
				return 'not_reached'
			# return with error
			elif (handle_base.get_error_code() > 0):
				print "error_code = " + str(handle_base.get_error_code())
				if ("light" in self.components):
					sss.set_light('red')
				return 'failed'

			# check if the base is moving
			loop_rate = rospy.Rate(freq) # hz
			if not self.is_moving: # robot stands still			
				# increase timers
				stopping_time += 1.0/freq
				announce_time += 1.0/freq

				# abort after timeout is reached
				if stopping_time >= self.timeout:
					sss.stop("base")
					if ("light" in self.components):
						sss.set_light('green')
					return 'not_reached'
			
				# announce warning after every 10 sec
				if announce_time >= 10.0:
					sss.say([self.warnings[random.randint(0,len(self.warnings)-1)]],False)
					announce_time = 0.0

				# set light to "thinking" after not moving for 2 sec
				if round(stopping_time) >= 2.0:
					if ("light" in self.components):
						sss.set_light("blue")
					yellow = False
			else:
				# robot is moving
				if not yellow:
					if ("light" in self.components):
						sss.set_light("yellow")
					yellow = True
		
			# sleep
			loop_rate.sleep()
