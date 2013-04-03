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
from cob_arm_navigation_python.MotionPlan import *

from pr2_python import transform_listener
from pr2_python import world_interface
from pr2_python import conversions
from pr2_python import hand_description

from copy import deepcopy
from tf.transformations import *
from tf_conversions import posemath as pm
import numpy

class Grasp(smach.State):
	PRESETS = {
	    'fixed_orientation_euler': (-1.706, 0.113, 2.278),
	    'grasp_offset': (0.02, 0.02, 0.1),
	    'pregrasp_offset': (0.20, 0.11, 0.1),
	    'lift_offset': (0,0,0.03),
	    'tcp_link': 'sdh_grasp_link',
	    'pregrasp_seed': 'pregrasp',
	}
	def __init__(self, presets = dict(), additional_input=[]):
		smach.State.__init__(self, 
			outcomes=['grasped','not_grasped','failed'],
			input_keys=['object']+additional_input, output_keys=['graspdata'])
		self.tl = transform_listener.get_transform_listener() # create singleton
		self.default_presets = PRESETS
		self.default_presets.update(presets)
	def execute(self, userdata, overwrite_presets = dict()):
		presets = self.default_presets
		presets.update(overwrite_presets)
		sss.set_light('blue')

		wi = WorldInterface()
		wi.reset_attached_objects()
		graspdata = dict()
		print userdata.object
		print "time difference = ", (rospy.Time.now() - userdata.object.pose.header.stamp).to_sec()


		# add wall
		wall_extent = [3.0,0.1,2.5]
		wall_pose = conversions.create_pose_stamped([  0, -0.99 - wall_extent[1], wall_extent[2]/2.0 ,0,0,0,1], '/base_link') # magic numbers are cool
		wi.add_collision_box(wall_pose, wall_extent, "wall")

		# add floor
		floor_extent = [3.0,3.0,0.1]
		floor_pose = conversions.create_pose_stamped([  0, 0, floor_extent[2]/2.0 ,0,0,0,1], '/base_link') # magic numbers are cool
		wi.add_collision_box(floor_pose, floor_extent, "floor")
		
		#transform into base_link
		grasp_pose = transform_listener.transform_pose_stamped('/base_link', userdata.object.pose, use_most_recent=False)
		print grasp_pose

		# add object bounding box
		obj_pose = deepcopy(userdata.object.pose)
		lwh = userdata.object.bounding_box_lwh
		m1 = pm.toMatrix( pm.fromMsg(obj_pose.pose) )
		m2 = pm.toMatrix( pm.fromTf( ((0,0, lwh.z/2.0),(0,0,0,1)) ) )
		obj_pose.pose = pm.toMsg( pm.fromMatrix(numpy.dot(m1,m2)) )
		wi.add_collision_box(obj_pose,(lwh.x*2.0,lwh.y*2.0,lwh.z) , "grasp_object")

		# compute minimal height of object bounding box edges in base_link
		min_z = grasp_pose.pose.position.z
		for k in [(0.5,0.5,1.0),(0.5,0.5,0.0),(0.5,-0.5,1.0),(0.5,-0.5,0.0),(-0.5,0.5,1.0),(-0.5,0.5,0.0),(-0.5,-0.5,1.0),(-0.5,-0.5,0.0)]:		
			m1 = pm.toMatrix( pm.fromMsg(grasp_pose.pose) ) # BFromO
			m2 = pm.toMatrix( pm.fromTf( ((k[0]*lwh.x,k[1]*lwh.y, k[2]*lwh.z),(0,0,0,1)) ) ) # inO
			min_z = min(min_z,pm.toMsg( pm.fromMatrix(numpy.dot(m1,m2))).position.z) #min_z inB

		# add table
		table_extent = (2.0, 2.0, min_z)
		# base_link
		# x - points towards front of robot
		# y - follow right hand rule
		# z - points upwards
		table_pose = conversions.create_pose_stamped([ -0.5 - table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1], '/base_link') # center of table, 0.5 meter behind the robot
		wi.add_collision_box(table_pose, table_extent, "table")

		# calculate grasp and lift pose
		grasp_pose.pose.position.x += presets['grasp_offset'][0]
		grasp_pose.pose.position.y += presets['grasp_offset'][1]
		grasp_pose.pose.position.z += presets['grasp_offset'][2]
		grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(*presets['fixed_orientation_euler'])) # orientation of sdh_grasp_link in base_link for 'grasp' joint goal
		
		graspdata['height'] =  grasp_pose.pose.position.z - table_extent[2]
		
		pregrasp_pose = deepcopy(grasp_pose)
		pregrasp_pose.pose.position.x += presets['pregrasp_offset'][0]
		pregrasp_pose.pose.position.y += presets['pregrasp_offset'][1]
		pregrasp_pose.pose.position.z += presets['pregrasp_offset'][2]

		lift_pose = deepcopy(grasp_pose)
		lift_pose.pose.position.x += presets['lift_offset'][0]
		lift_pose.pose.position.y += presets['lift_offset'][1]
		lift_pose.pose.position.z += presets['lift_offset'][2]
		
		mp = MotionPlan()
		# open hand
		mp += CallFunction(sss.move, 'sdh','cylopen', False)
		mp += MoveArm('arm',[pregrasp_pose,[presets['tcp_link']]], seed = presets['pregrasp_seed'])
		mp += MoveComponent('sdh','cylopen', True)

		# allow collison hand/object
		for l in hand_description.HandDescription('arm').touch_links:
		    mp += EnableCollision("grasp_object", l)
		
		# goto grasp
		mp += MoveArm('arm', [grasp_pose,[presets['tcp_link']]]) # Move sdh_grasp_link to grasp_pose
		
		# close hand
		mp += MoveComponent('sdh','cylclosed')
		
		# check grasp
		#mp += CheckService('/sdh_controller/is_cylindric_grasped', Trigger, lambda res: res.success.data)
		
		# disable collison
		mp += ResetCollisions()

		# attach object
		mp += AttachObject('arm', "grasp_object")
		
		# enable collision
		mp += EnableCollision("grasp_object", "table")
		
		# lift motion
		mp += MoveArm('arm', [lift_pose,[presets['tcp_link']]]) # Move sdh_grasp_link to lift_pose

		# disable collison
		mp += ResetCollisions()

		# move away
		#mp += MoveArm('arm', [pregrasp_pose,[presets['tcp_link']]])
		
		# goto hold
		mp += MoveArm('arm', 'hold')
		
		userdata.graspdata = graspdata

		if not mp.plan(5).success:
			sss.set_light('green')
			return "not_grasped"
		
		sss.set_light('yellow')
		sss.say(["I am grasping " + userdata.object.label + " now."],False)
		# run, handle errors
		i = 0
		for ex in mp.execute():
			if not ex.wait(80.0).success:
				sss.set_light('red')
				return 'failed'
			i+=1
		sss.set_light('green')
		return 'grasped'

class GraspExternalPresets(smach.State):
	def __init__(self, presets = dict()):
		Grasp.__init__(self,presets,['graspdata'])
	def execute(self, userdata):
	    return Grasp.execute(self.userdata,self.userdata.graspdata)