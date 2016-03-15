import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from cob_srvs.srv import Trigger

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *

from cob_arm_navigation_python.MoveHand import *
import cob_arm_navigation_python.MoveArm
from cob_arm_navigation_python.MotionPlan import *

from pr2_python import world_interface
from pr2_python import conversions

from copy import deepcopy
from tf.transformations import *
from tf_conversions import posemath as pm
import numpy

class MoveArm(smach.State):
	def __init__(self, *args, **kwargs):
		smach.State.__init__(self, 
			outcomes=['succeeded','not_succeeded','failed'])
		self.mp = MotionPlan()
		# move arm
		self.mp += cob_arm_navigation_python.MoveArm.MoveArm(*args, **kwargs)
	def execute(self, userdata):
		sss.set_light('blue')

		wi = WorldInterface()
		wi.reset_attached_objects()

                # add floor
                floor_extent = [3.0,3.0,0.1]
		floor_pose = conversions.create_pose_stamped([  0, 0, floor_extent[2]/2.0 ,0,0,0,1], '/base_link') # magic numbers are cool
		wi.add_collision_box(floor_pose, floor_extent, "floor")
		
		if not self.mp.plan(5).success:
                        sss.set_light('green')
			return "not_succeeded"
		
                sss.set_light('yellow')
		# run, handle errors
		i = 0
		for ex in self.mp.execute():
			if not ex.wait(80.0).success:
                                sss.set_light('red')
				return 'failed'
			i+=1
                sss.set_light('green')
		return 'succeeded'

class MoveArmUnplanned(MoveArm):
	def __init__(self, *args, **kwargs):
		smach.State.__init__(self, 
			outcomes=['succeeded','not_succeeded','failed'])
		self.mp = MotionPlan()
		# move arm
		self.mp += cob_arm_navigation_python.MoveArm.MoveArmUnplanned(*args, **kwargs)	
