#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_gate_keeper')
import rospy

import random

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from cob_gate_keeper.msg import *

def gate_keeper_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('gate_keeper', GateKeeperAction)

    # Waits until the action server has started up and started
    # listening for goals.
    if not client.wait_for_server(rospy.Duration(5)):
        return 'failed'

    client.cancel_all_goals()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gate_keeper_cancel' + str(random.randint(1,1000000000)))
        result = gate_keeper_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"