import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import tf
from simple_script_server import *  # import script
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *

from cob_arm_navigation_python.MoveHand import *
from cob_arm_navigation_python.MoveArm import *
from cob_arm_navigation_python.MotionPlan import *

from pr2_python import transform_listener
from pr2_python import world_interface
from pr2_python import hand_description
from pr2_python import conversions

from copy import deepcopy

## Put object on tray side state
#
# This state puts a side grasped object on the tray
class PutObjectOnTray(smach.State):

	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['put', 'not_put', 'failed'],
			input_keys=['object','graspdata'])

	def execute(self, userdata):
		sss.set_light('blue')
		sss.move("head","front", False)
		#TODO select position on tray depending on how many objects are on the tray already

		""" wi = WorldInterface()

		# add tray block
		block_extent = [0.27,0.38,0.892]
		block_pose = conversions.create_pose_stamped([ 0.601 - block_extent[0]/2.0, -0.120 + block_extent[1]/2.0, 0.892/2.0 ,0,0,0,1], 'base_link') # magic numbers are cool
		wi.add_collision_box(block_pose, block_extent, "block")

		pos_x, pos_y, pos_z = [0.601, -0.120, 0.892] # tray_right_link in base_link		
		pos_x -= 0.13
		pos_y += 0.065
		pos_z += userdata.graspdata['height'] + 0.04
		
		[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.5708,0,0) # rpy TODO: check orientation
		put_pose = conversions.create_pose_stamped([pos_x, pos_y, pos_z,new_x, new_y, new_z, new_w], 'base_link')
		
		lift_pose = deepcopy(put_pose)
		lift_pose.pose.position.y -= 0.2
		lift_pose.pose.position.z += 0.1
		
		mp = MotionPlan()
		
		# move arm to lift
		mp += CallFunction(sss.move, 'tray','up', False)
		mp += MoveArmUnplanned('arm', 'intermediateback')
		mp += MoveArm('arm', [lift_pose,['sdh_grasp_link']], seed = 'overtray')
		
		# allow collison tray/object
		# mp += EnableCollision("grasp_object", "tray_link")

		# tray up
		mp += MoveComponent('tray', 'up', True)

		# move to put
		mp += MoveArmUnplanned('arm', [put_pose,['sdh_grasp_link']])
		
		# disable collisions
		# mp += ResetCollisions()

		# open hand
		mp += MoveComponent('sdh', 'cylopen')

		# check free
		#mp += CheckService('/sdh_controller/one_pad_contact', Trigger, lambda res: not res.success.data)
		
		# allow collison hand/object
		#for l in HandDescription('arm').touch_links:
		#    mp += EnableCollision("grasp_object", l)
		
		# move arm to lift
		mp += MoveArm('arm', [lift_pose,['sdh_grasp_link']])

		# disable collisions
		#mp += ResetCollisions()

		# detach object
		mp += DetachObject('arm', "grasp_object")

		mp += CallFunction(sss.move, 'sdh','home', False)
		# move arm to folded
		mp += MoveArmUnplanned('arm', 'tray-to-folded')

		if not mp.plan(5).success:
			sss.set_light('green')
			return "not_put"

		sss.set_light('yellow')
		for ex in mp.execute():
			if not ex.wait(80.0).success:
				sss.set_light('red')
				return 'failed'  """

		# quick fix: non planned movements without ik
		sss.set_light('yellow')
		handle_arm = sss.move("arm","grasp-to-tray",False)
		sss.move("tray","up")
		handle_arm.wait()
		sss.move("sdh","cylopen")
		handle_arm = sss.move("arm","tray-to-folded",False)
		rospy.sleep(3)
		sss.move("sdh","cylclosed")
		handle_arm.wait()
		# end fix

		sss.set_light('green')
		return 'put'
