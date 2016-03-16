#!/usr/bin/python

import rospy
import smach
import smach_ros

import random
from nav_msgs.srv import *
from cob_object_detection_msgs.msg import *

from cob_generic_states_experimental.ApproachPose import *
from cob_generic_states_experimental.DetectObjectsFrontside import *

class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['selected','not_selected','failed'],
			output_keys=['base_pose'])
			
		self.goals = []
			
	def execute(self, userdata):
		# defines
		x_min = 0
		x_max = 4.0
		x_increment = 2
		y_min = -4.0
		y_max = 0.0
		y_increment = 2
		th_min = -3.14
		th_max = 3.14
		th_increment = 2*3.1414926/4
		
		# generate new list, if list is empty
		if len(self.goals) == 0:	
			x = x_min
			y = y_min
			th = th_min
			while x <= x_max:
				while y <= y_max:
					while th <= th_max:
						pose = []
						pose.append(x) # x
						pose.append(y) # y
						pose.append(th) # th
						self.goals.append(pose)
						th += th_increment
					y += y_increment
					th = th_min
				x += x_increment
				y = y_min
				th = th_min

		#print self.goals
		#userdata.base_pose = self.goals.pop() # takes last element out of list
		userdata.base_pose = self.goals.pop(random.randint(0,len(self.goals)-1)) # takes random element out of list

		return 'selected'

class AnnounceFoundObjects(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['announced','not_announced','failed'],
			input_keys=['objects'],
			output_keys=['objects'])
			
	def execute(self, userdata):
		object_names = ""
		for obj in userdata.objects:
			object_names += obj.label + ", "
		
		if object_names != "":
			sss.say(["I found: " + object_names])
		else:
			sss.say(["I found: nothing"])
		userdata.objects = []
		return 'announced'

class Explore(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('SELECT_GOAL',SelectNavigationGoal(),
                                   transitions={'selected':'MOVE_BASE',
                                                'not_selected':'finished',
                                                'failed':'failed'})
            
            smach.StateMachine.add('MOVE_BASE',ApproachPose(),
                                   transitions={'reached':'DETECT',
                                                'not_reached':'SELECT_GOAL',
                                                'failed':'failed'})

            smach.StateMachine.add('DETECT',DetectObjectsFrontside(['milk','pringles'],mode="one"),
                                   transitions={'detected':'ANNOUNCE',
                                                'not_detected':'ANNOUNCE',
                                                'failed':'failed'})

            smach.StateMachine.add('ANNOUNCE',AnnounceFoundObjects(),
                                   transitions={'announced':'SELECT_GOAL',
                                                'not_announced':'SELECT_GOAL',
                                                'failed':'failed'})


















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Explore(),
            		transitions={'finished':'ended',
            					'failed':'ended'})

if __name__=='__main__':
	rospy.init_node('Explore')
	sm = SM()
	sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
	sis.start()
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
