#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: cob_gate_keeper
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: August 2012
#
# \brief
# Implements A Queue Version of the Action Server
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY  without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
# Based on Python simple_action_server.py by Alexander Sorokin.

from __future__ import with_statement

import roslib 
roslib.load_manifest('actionlib')
import rospy

import threading
import traceback

from actionlib_msgs.msg import *

from actionlib import ActionServer
from actionlib.server_goal_handle import ServerGoalHandle
import collections

import Queue

def nop_cb(goal_handle):
	pass


class deque(collections.deque):
	def __init__(self, iterable=(), maxlen=None):
		super(deque, self).__init__(iterable, maxlen)
		self._maxlen = maxlen
	@property
	def maxlen(self):
		return self._maxlen

## @class QueuedActionServer
## @brief The QueuedActionServer
## implements a queued goal policy on top of the ActionServer class. The
## specification of the policy is as follows: posterior goals are only
## executed when the current running goal is sucessfully executed

class QueuedActionServer:
    ## @brief Constructor for a QueuedActionServer
    ## @param name A name for the action server
    ## @param execute_cb Optional callback that gets called in a separate thread whenever
    ## a new goal is received, allowing users to have blocking callbacks.
    ## Adding an execute callback also deactivates the goalCallback.
    ## @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.

	def __init__(self, name, ActionSpec, execute_cb = None, auto_start = True):

		self.new_goal = False
		self.preempt_request = False
		self.new_goal_preempt_request = False
		self.maxlen = 5

		#self.goals_buf = deque(maxlen=5)
		self.goals_buf = Queue.Queue(maxsize=self.maxlen)
		self.current_indexA = 0
		self.current_indexP = 0

		#self.maxlen = self.goals_buf.maxlen

		self.execute_callback = execute_cb
		self.goal_callback = None
		self.preempt_callback = None

		self.need_to_terminate = False
		self.terminate_mutex = threading.RLock()
		self.lock = threading.RLock()

		self.execute_condition = threading.Condition(self.lock)

		self.current_goal = ServerGoalHandle()
		self.next_goal = ServerGoalHandle()

		if self.execute_callback:
			self.execute_thread = threading.Thread(None, self.executeLoop)
			self.execute_thread.start()
		else:
			self.execute_thread = None

		#create the action server
		self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback,self.internal_preempt_callback,auto_start)


	def __del__(self):
		if hasattr(self, 'execute_callback') and self.execute_callback:
			with self.terminate_mutex:
				self.need_to_terminate = True

			assert(self.execute_thread)
			self.execute_thread.join()


    ## @brief Accepts a new goal when one is available The status of this
    ## goal is set to active upon acceptance, and the status of any
    ## previously active goal is set to preempted. Preempts received for the
    ## new goal between checking if isNewGoalAvailable or invokation of a
    ## goal callback and the acceptNewGoal call will not trigger a preempt
    ## callback.  This means, isPreemptReqauested should be called after
    ## accepting the goal even for callback-based implementations to make
    ## sure the new goal does not have a pending preempt request.
    ## @return A shared_ptr to the new goal.
	def accept_new_goal(self):

		
		with self.lock:
			if not self.new_goal or not self.next_goal.get_goal():
				rospy.logerr("Attempting to accept the next goal when a new goal is not available")
				return None

			rospy.logdebug("Accepting a new goal")

			#accept the next goal
			self.current_goal = self.next_goal
			self.new_goal = False

			#set preempt to request to equal the preempt state of the new goal
			self.preempt_request = self.new_goal_preempt_request
			self.new_goal_preempt_request = False

			#set the status of the current goal to be active
			self.current_goal.set_accepted("This goal has been accepted by the queued action server")

			return self.current_goal.get_goal()


    ## @brief Allows  polling implementations to query about the availability of a new goal
    ## @return True if a new goal is available, false otherwise
	def is_new_goal_available(self):
		return self.new_goal


    ## @brief Allows  polling implementations to query about preempt requests
    ## @return True if a preempt is requested, false otherwise
	def is_preempt_requested(self):
		return self.preempt_request

    ## @brief Allows  polling implementations to query about the status of the current goal
    ## @return True if a goal is active, false otherwise
	def is_active(self):
		if not self.current_goal.get_goal():
			return False

		status = self.current_goal.get_goal_status().status
		return status == actionlib_msgs.msg.GoalStatus.ACTIVE or status == actionlib_msgs.msg.GoalStatus.PREEMPTING

    ## @brief Sets the status of the active goal to succeeded
    ## @param  result An optional result to send back to any clients of the goal
	def set_succeeded(self,result=None, text=""):
		with self.lock:
			if not result:
				result=self.get_default_result()
			self.current_goal.set_succeeded(result, text)

    ## @brief Sets the status of the active goal to aborted
    ## @param  result An optional result to send back to any clients of the goal
	def set_aborted(self, result = None, text=""):
		with self.lock:
			if not result:
				result=self.get_default_result()
			self.current_goal.set_aborted(result, text)

    ## @brief Publishes feedback for a given goal
    ## @param  feedback Shared pointer to the feedback to publish
	def publish_feedback(self,feedback):
		self.current_goal.publish_feedback(feedback)


	def get_default_result(self):
		return self.action_server.ActionResultType()

    ## @brief Sets the status of the active goal to preempted
    ## @param  result An optional result to send back to any clients of the goal
	def set_preempted(self,result=None, text=""):
		if not result:
			result=self.get_default_result()
		with self.lock:
			rospy.logdebug("Setting the current goal as canceled")
			self.current_goal.set_canceled(result, text)

    ## @brief Allows users to register a callback to be invoked when a new goal is available
    ## @param cb The callback to be invoked
	def register_goal_callback(self,cb):
		if self.execute_callback:
			rospy.logwarn("Cannot call QueuedActionServer.register_goal_callback() because an executeCallback exists. Not going to register it.")
		else:
			self.goal_callback = cb

    ## @brief Allows users to register a callback to be invoked when a new preempt request is available
    ## @param cb The callback to be invoked
	def register_preempt_callback(self, cb):
		self.preempt_callback = cb
	

    ## @brief Explicitly start the action server, used it auto_start is set to false
	def start(self):
		self.action_server.start()


    ## @brief Callback for when the ActionServer receives a new goal and passes it on
	def internal_goal_callback(self, goal):

		self.execute_condition.acquire()
		try:

			rospy.logdebug("A new goal %shas been recieved by the Queued goal action server",goal.get_goal_id().id)

			if(self.goals_buf.empty()):
				self.new_goal = True
				self.next_goal = goal
				self.goals_buf.put(goal, timeout=1)
			else:
				
				self.goals_buf.put(goal, timeout=1)

			
			rospy.loginfo("Queued New Goal")

			if self.goal_callback:
				self.goal_callback()

			#rospy.loginfo("Goals List-----------------------------------------------")

			#for item in self.goals_buf:		

			#	rospy.loginfo("Goals Buffer%s" %item.get_goal_status())

			#rospy.loginfo("End of the Goals List-------------------------------------")

	#if the user has defined a goal callback, we'll call it now
			
	#Trigger runLoop to call execute()
			self.execute_condition.notify()
			self.execute_condition.release()
	
	
		except Exception, e:
			rospy.logerr("QueuedActionServer.internal_goal_callback - exception %s",str(e))
			self.execute_condition.release()

    ## @brief Callback for when the ActionServer receives a new preempt and passes it on
	def internal_preempt_callback(self,preempt):
		with self.lock:
			rospy.logdebug("A preempt has been received by the QueuedActionServer")

			#if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
			if(preempt == self.current_goal):
				rospy.logdebug("Setting preempt_request bit for the current goal to TRUE and invoking callback")
				self.preempt_request = True

			#if the user has registered a preempt callback, we'll call it now
				if(self.preempt_callback):
					self.preempt_callback()
			#if the preempt applies to the next goal, we'll set the preempt bit for that
			elif(preempt == self.next_goal):
				rospy.logdebug("Setting preempt request bit for the next goal to TRUE")
				self.new_goal_preempt_request = True

    ## @brief Called from a separate thread to call blocking execute calls
	def executeLoop(self):
		loop_duration = rospy.Duration.from_sec(.1)

		while (not rospy.is_shutdown()):
			rospy.logdebug("SAS: execute")
			
			shall_run = False
			with self.lock:

				if (self.is_active()):
					rospy.loginfo("Still Running Current GOAL!!") 
					pass
				elif (self.is_new_goal_available()):
					goal = self.accept_new_goal() 
					if not self.execute_callback:
  						rospy.logerr("execute_callback_ must exist. This is a bug in QueuedActionServer") 
  						return
					shall_run = True

			if shall_run:
                  		try:
                      			self.execute_callback(goal)

					if self.is_active():
						rospy.logwarn("Your executeCallback did not set the goal to a terminal status.  " +
							"This is a bug in your ActionServer implementation. Fix your code!  "+
							"For now, the ActionServer will set this goal to aborted")
						self.set_aborted(None, "No terminal state was set.")
		
				except Exception, ex:
					rospy.logerr("Exception in your execute callback: %s\n%s", str(ex),
						traceback.format_exc())
					self.set_aborted(None, "Exception in execute callback: %s" % str(ex))



			with self.terminate_mutex:
				if (self.need_to_terminate):
					break
					
			with self.execute_condition:
				self.execute_condition.wait(loop_duration.to_sec())



