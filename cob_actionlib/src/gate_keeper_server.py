#! /usr/bin/env python

import roslib  
roslib.load_manifest('cob_gate_keeper')
import rospy
import queued_action_server as mal
import actionlib

from cob_gate_keeper.msg import *

import Queue

Empty = Queue.Empty

class GateKeeperActionClass(object):
# create messages that are used to publish feedback/result
	_feedback = GateKeeperFeedback()
	_result   = GateKeeperResult()

	def __init__(self, name):
		self._action_name = name
		self._as = mal.QueuedActionServer(self._action_name, GateKeeperAction, execute_cb=self.execute_cb, auto_start=False)
		rospy.loginfo("start")
		self._as.start()
		self.count = 0
		

	def execute_cb(self, goal):
		# helper variables
		r = rospy.Rate(1)
		
		success = True

# publish info to the console for the user
		rospy.loginfo('%s: Started' % self._action_name)
		
	        while not rospy.is_shutdown():
			self.count = self.count + 1

			if (self._as.is_active()):
				success = False	
				rospy.logerr("Current Goal is still active. Counting until 10: %s", self.count)

				if(self._as.is_preempt_requested()):
					rospy.loginfo('%s: Preempted' % self._action_name)
					self._as.set_preempted()
					
					break

			elif (self._as.is_new_goal_available()):
				goal = self._as.accept_new_goal()
				rospy.loginfo("New Goal???%s", self._as.new_goal)
				

			
				
			if(self.count == 10):
				try:
					if(self._as.goals_buf.empty()):
						self.count = 0
						if(self._as.is_active()):
							success = True
						else:	
							break
					else:
				
						success = True
						self._as.next_goal = self._as.goals_buf.get(timeout=1)
						self._as.new_goal = True
					
						self.count = 0
				
				
			

					if success:
						rospy.loginfo("Entered the terminal state of the callback")
						#self._result.sequence = self._feedback.sequence
						rospy.loginfo('%s: Succeeded' % self._action_name)
						self._as.set_succeeded(self._result)

				except Empty:
					self.count = 0
					rospy.logerr("No more next goals at the moment\n")

			r.sleep()

if __name__ == '__main__':
  rospy.init_node('gate_keeper')
  GateKeeperActionClass(rospy.get_name())
  rospy.spin()
