#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import struct
import ctypes
import copy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from cob_3d_mapping_msgs.srv import GetApproachPoseForPolygon
from cob_3d_mapping_msgs.msg import *
from sensor_msgs.msg import PointCloud2, PointField  
from std_msgs.msg import Header

import tf
from tf.transformations import *

from ApproachPose import *

from ScreenFormatting import *


"""Computes all accessible robot poses around polygon, returns best pose (successively)"""
class ComputeNavigationGoals(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'failed'],
			input_keys=['polygon','new_computation_flag'],
			output_keys=['goal_poses','new_computation_flag'])

	def execute(self, userdata):
		sf = ScreenFormat("ComputeNavigationGoals")
		if not userdata.new_computation_flag:
			return 'computed'
		rospy.wait_for_service('map_accessibility_analysis/map_polygon_accessibility_check',10)
		try:
			get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_polygon_accessibility_check', GetApproachPoseForPolygon)
			res = get_approach_pose(userdata.polygon)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		userdata.goal_poses = res.approach_poses.poses
		userdata.new_computation_flag = False
		return 'computed'
	
	
class SelectNavigationGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['computed', 'no_goals_left', 'failed'],
			input_keys=['goal_poses'],
			output_keys=['goal_pose'])
		self.listener = tf.TransformListener(True, rospy.Duration(20.0))
		self.nogo_area_radius_squared = 1*1 #in meters, radius the current goal covers

	def execute(self, userdata):
		sf = ScreenFormat("SelectNavigationGoal")
		"""compute closest position to current robot pose"""
		try:
			t = rospy.Time(0)
			self.listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
			robot_pose = self.listener.lookupTransform('/map', '/base_link', t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print "Could not lookup robot pose: %s" %e
			return 'failed'
		closest_pose = Pose()
		minimum_distance_squared = 100000.0
		found_valid_pose = 0
		for pose in userdata.goal_poses:
			dist_squared = (robot_pose[0][0]-pose.position.x)*(robot_pose[0][0]-pose.position.x)+(robot_pose[0][1]-pose.position.y)*(robot_pose[0][1]-pose.position.y)
			if dist_squared < minimum_distance_squared:
				minimum_distance_squared = dist_squared
				closest_pose = copy.deepcopy(pose)
				found_valid_pose = 1
		if found_valid_pose == 0:
			return 'no_goals_left'
		
		"""delete all poses too close to current goal"""
		for p in range(len(userdata.goal_poses)-1,-1,-1):
			pose = userdata.goal_poses[p]
			dist_squared = (closest_pose.position.x-pose.position.x)*(closest_pose.position.x-pose.position.x)+(closest_pose.position.y-pose.position.y)*(closest_pose.position.y-pose.position.y)
			if dist_squared < self.nogo_area_radius_squared:
				userdata.goal_poses.remove(pose)
		
		"""convert goal pose"""
		[roll, pitch, yaw] = euler_from_quaternion([closest_pose.orientation.x,
												closest_pose.orientation.y,
												closest_pose.orientation.z,
												closest_pose.orientation.w])
		userdata.goal_pose=[closest_pose.position.x, closest_pose.position.y, yaw+math.pi]
		return 'computed'



class ApproachPolygon(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,
			outcomes=['reached', 'not_reached', 'failed'],
			input_keys=['polygon','new_computation_flag'],
			output_keys=['new_computation_flag'])
		with self:

			smach.StateMachine.add('COMPUTE_GOALS', ComputeNavigationGoals(),
									transitions={'computed':'SELECT_GOAL',
												'failed':'failed'})
			
			smach.StateMachine.add('SELECT_GOAL', SelectNavigationGoal(),
									transitions={'computed':'MOVE_BASE',
												'no_goals_left':'not_reached',
												'failed':'failed'})

			smach.StateMachine.add('MOVE_BASE', ApproachPose(),
									transitions={'reached':'reached',
												'not_reached':'SELECT_GOAL',
												'failed':'failed'},
									remapping = {'base_pose':'goal_pose'})


_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
	fmt = '>' if is_bigendian else '<'
	
	offset = 0
	for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
		if offset < field.offset:
			fmt += 'x' * (field.offset - offset)
			offset = field.offset
		if field.datatype not in _DATATYPES:
			print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
		else:
			datatype_fmt, datatype_length = _DATATYPES[field.datatype]
			fmt    += field.count * datatype_fmt
			offset += field.count * datatype_length

	return fmt

def create_cloud(header, fields, points):
	"""
	Create a L{sensor_msgs.msg.PointCloud2} message.
	
	@param header: The point cloud header.
	@type  header: L{std_msgs.msg.Header}
	@param fields: The point cloud fields.
	@type  fields: iterable of L{sensor_msgs.msg.PointField}
	@param points: The point cloud points.
	@type  points: list of iterables, i.e. one iterable for each point, with the
	               elements of each iterable being the values of the fields for
	               that point (in the same order as the fields parameter)
	@return: The point cloud.
	@rtype:  L{sensor_msgs.msg.PointCloud2}
	"""
	
	cloud_struct = struct.Struct(_get_struct_fmt(False, fields))
	
	buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
	
	point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
	offset = 0
	
	for p in points:
		pack_into(buff, offset, *p)
		offset += point_step
	return PointCloud2(header=header,
						height=1,
						width=len(points),
						is_dense=False,
						is_bigendian=False,
						fields=fields,
						point_step=cloud_struct.size,
						row_step=cloud_struct.size * len(points),
						data=buff.raw)


if __name__ == '__main__':
	try:
		rospy.init_node("approach_polygon")
		sm = ApproachPolygon()
		sm.userdata.polygon = cob_3d_mapping_msgs.msg.Shape()
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = '/map'
		print "1"
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
					PointField('y', 4, PointField.FLOAT32, 1),
					PointField('z', 8, PointField.FLOAT32, 1)]
		print "2"
		#points = [[1.0,-2.0,0.0], [1.0,-3.0,0.0], [2.0,-3.0,0.0], [2.0,-2.0,0.0]];
		points = ((1.0,-2.0,0.0), (1.0,-3.0,0.0), (2.0,-3.0,0.0), (2.0,-2.0,0.0));
		print "3"
		pc_msg = create_cloud(header, fields, points)
		print "4"
		sm.userdata.polygon.points.append(pc_msg)
		sm.userdata.polygon.holes.append(False)
		sm.userdata.new_computation_flag = True

		# introspection -> smach_viewer
		sis = smach_ros.IntrospectionServer('map_accessibility_analysis_introspection', sm, '/MAP_ACCESSIBILITY_ANALYSIS')
		sis.start()
		
		sm.execute()
		rospy.spin()
		sis.stop()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)