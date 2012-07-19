#!/usr/bin/python

import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class MoveYourself(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['succeeded','failed'])

	def execute(self, userdata):
		# ------------------------------------
		handle_arm = sss.move("arm","pregrasp",False)
		handle_tray = sss.move("tray","up",False)
		handle_torso = sss.move("torso","nod",False)
		handle_sdh = sss.move("sdh","cylopen",False)
		handle_head = sss.move("head","back",False)
		
		handle_head.wait()
		handle_sdh.wait()
		handle_torso.wait()
		handle_tray.wait()
		handle_arm.wait()
		
		if handle_head.get_state() != 3 or handle_sdh.get_state() != 3 or handle_torso.get_state() != 3 or	handle_tray.get_state() != 3 or handle_arm.get_state() != 3:
			print "head state = ",handle_head.get_state() 
			print "sdh state = ",handle_sdh.get_state()
			print "torso state = ",handle_torso.get_state()
			print "tray state = ",handle_tray.get_state()
			print "arm state = ",handle_arm.get_state()
			return 'failed' 
		
		# ------------------------------------
		handle_arm = sss.move("arm","folded",False)
		handle_tray = sss.move("tray","down",False)
		handle_torso = sss.move("torso","shake",False)
		handle_sdh = sss.move("sdh","cylclosed",False)
		handle_head = sss.move("head","front")
		
		handle_head.wait()
		handle_sdh.wait()
		handle_torso.wait()
		handle_tray.wait()
		handle_arm.wait()

		if handle_head.get_state() != 3 or handle_sdh.get_state() != 3 or handle_torso.get_state() != 3 or	handle_tray.get_state() != 3 or handle_arm.get_state() != 3:
			print "head state = ",handle_head.get_state() 
			print "sdh state = ",handle_sdh.get_state()
			print "torso state = ",handle_torso.get_state()
			print "tray state = ",handle_tray.get_state()
			print "arm state = ",handle_arm.get_state()
			return 'failed'

		return 'succeeded'



















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',MoveYourself(),
                    transitions={'succeeded':'ended',
                                'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('MoveYourself')
    sm = SM()
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
