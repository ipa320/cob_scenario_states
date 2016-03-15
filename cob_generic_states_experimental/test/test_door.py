#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy

import sensor_msgs.msg
from sensor_msgs.msg import *

from cob_generic_states_experimental.srv import Door

import subprocess
from subprocess import Popen, PIPE, STDOUT

import os
import sys
import unittest


import rostest

import threading
import re
import random


# global variables
global Read
Read = False
global bag_path

import logging
################################################################### runbag class thread 
class MyThread1 ( threading.Thread ): # bag thread

    def run ( self ):
        global time_limit,time
        os.system("killall -9 play") #kill all bag
        subprocess.Popen("rosbag play  -l "+bag_path, shell=True) # run bag	
        while not Read :
            rospy.sleep(0.1)	
        print "hoho"
        os.system("killall -9 play") #stop bag
		

###################################################################  Class test
class PythonAPITest(unittest.TestCase):

    # init
    def __init__(self, *args):
        super(PythonAPITest, self).__init__(*args)
        rospy.init_node('test_door',anonymous=True)
        # self.ns_global_prefix = "test_door"
		
    def test_python_api(self):

        global bag_path,Read
        try:
            # get test parameter
            #if not rospy.has_param('~object_number'):
            #    self.fail('Parameter face_number does not exist on ROS Parameter Server')
            #object_number = rospy.get_param('~object_number')		
            # get test parameter
            if not rospy.has_param('~bag_path'):
                self.fail('Parameter bag_path does not exist on ROS Parameter Server')
            bag_path = rospy.get_param('~bag_path')		
            if not rospy.has_param('~ndoors'):
                self.fail('Parameter ndoors does not exist on ROS Parameter Server')
            ndoors = rospy.get_param('~ndoors')		
            
	except KeyError, e:
            self.fail('Parameters not set properly')


        # play bag
        t = MyThread1()
        t.start()
        #MyThread1().start() #play bag
        #rospy.loginfo(random.uniform(0, 5))		
       	#rospy.sleep(random.uniform(0, 5))#random waiting time to test the bag in different moment ( if bag = 5 sec (0,5))

        # call service
        print "hi"
        rospy.sleep(5)
        try:
            rospy.wait_for_service('/door_state', 2)
        except Exception, e:
            print '/door_state service not found!'
            self.fail("Service not available")

        self.id()
        print "hi"
        print "hi ndoors: " + str(ndoors)
        try:
            print "hi\n\n"
            doorFunc = rospy.ServiceProxy('/door_state',Door)  # launch detect
            res = doorFunc()
            print res
            Read = True
        except Exception, e:
            self.fail("Service detect object unavailable")	


        # checks:
        doors = res.doors
        if len(doors) != ndoors:
            self.fail("Wrong number of recognized doors, should be " + str(ndoors) + ", detected = " + str(len(doors)))
        if len(doors) == ndoors:
            for i in range(0, len(doors)):
                print doors[i]


        # Stop rospy						
        def myhook(): 
            print "shutdown time!"
            logging.info('shutdown')
            rospy.on_shutdown(myhook)
			
################################################################### start the rostest
if __name__ == '__main__':
    #logging.basicConfig( stream=sys.stderr )
    #logging.getLogger( "SomeTest.testSomething" ).setLevel( logging.DEBUG )
    logging.basicConfig(filename='/home/tys/git/care-o-bot/cob_scenario_states/cob_generic_states_experimental/test/example.log',level=logging.DEBUG)
    logging.debug('This message should go to the log file')
    logging.info('So should this')
    logging.warning('And this, too')
    #unittest.main()

    try:
        rostest.run('rostest', 'door_test', PythonAPITest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    os.system("killall -9 play") #stop bag if still running
    print "exiting"

