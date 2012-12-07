#! /usr/bin/env python

import roslib  
roslib.load_manifest('cob_gate_keeper')
import rospy

import random

import actionlib
import queued_action_client as  mal
# Brings in the QueuedActionClient

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from cob_gate_keeper.msg import *

def gate_keeper_client():

    rospy.loginfo("1")
    # Creates the QueuedActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = mal.QueuedActionClient('gate_keeper', GateKeeperAction)
    rospy.loginfo("2")
    # Waits until the action server has started up and started
    # listening for goals.
    if not client.wait_for_server(rospy.Duration(5)):
        return 'failed'
    rospy.loginfo("3")
    # Creates a goal to send to the action server.
    goal = GateKeeperGoal()
    rospy.loginfo("4")
    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.loginfo("5")
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.loginfo("6")
    # Prints out the result of executing the action
    rospy.loginfo("state = %s" %client.get_state())
    rospy.loginfo("7")
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the QueuedActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gate_keeper_client' + str(random.randint(1,1000000000)))
        result = gate_keeper_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
