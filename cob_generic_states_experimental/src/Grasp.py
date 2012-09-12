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
from cob_arm_navigation_python.MoveArm import *
from cob_arm_navigation_python.MovePlan import *

from pr2_python import transform_listener
from pr2_python import world_interface
from pr2_python import conversions

from copy import deepcopy
from tf.transformations import *

class Grasp(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['grasped','not_grasped','failed'],
			input_keys=['object'])
	def execute(self, userdata):
		
		wi = WorldInterface()
		table_extent = (2.0, 2.0, 0.74)
		table_pose = conversions.create_pose_stamped([ 0.5 + table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1])
		wi.add_collision_box(table_pose, table_extent, "table")
		# add table
		# (add floor)
		
		#transform into base_link
		grasp_pose = transform_listener.transform_pose_stamped('base_link', userdata.object.pose)
		
		obj_pose = deepcopy(grasp_pose)
		# add object bounding box
		with userdata.object.bounding_box_lwh:
			obj_pose.pose.position.z += z/2.0
			wi.add_collision_box(obj_pose,(x,y,z) , "grasp_object")
		
		# calculate grasp and lift pose
		grasp_pose.pose.position.x += 0.03
		grasp_pose.pose.position.y += 0.03
		grasp_pose.pose.position.z += 0.03
		grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(-1.581, -0.019, 2.379))
		
		lift_pose = deepcopy(grasp_pose)
		lift_pose.pose.position.z += 0.03
		
		mp = MotionPlan()
		# open hand
		mp += MoveComponent('sdh','cylopen')
		
		# goto grasp
		mp += MoveArm('arm', [grasp_pose,['sdh_grasp_link']])
		
		# close hand
		mp += MoveComponent('sdh','cylclosed')
		
		# check grasp
		mp += CheckService('/sdh_controller/is_cylindric_grasped', Trigger, lambda res: return res.success.data)
		
		# attach object
		mp += AttachObject('arm', "grasp_object")
		
		# enable collision
		mp += EnableCollision("grasp_object", "table")
		
		# lift motion
		mp += MoveArm('arm', [lift_pose,['sdh_grasp_link']])
		
		# disable collison
		mp += ResetCollisions()
		
		# goto hold
		mp += MoveArm('arm', 'hold')
		
		if not mp.plan().success:
			return "not_grasped"
		
		sss.say(["I am grasping " + userdata.object.label + " now."])
		# run, handle errors
		for ex in mp.execute():
			if not ex.wait().success:
				return 'failed'
		return 'grasped'
