import abc
from abc_skill import SkillsBase
import yaml

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
import random

import condition_check
import skill_state_approachpose

import tf
from tf.msg import tfMessage 
from tf.transformations import euler_from_quaternion

import skill_state_detectobjectsfront

class SkillImplementation(SkillsBase):

	def __init__(self, object_names = ['milk','pringles']):

		rospy.loginfo("Executing the detect object fronts Machine")
                smach.StateMachine.__init__(self,outcomes=['ended'], output_keys=['objects'])
		rospy.set_param("detect_object_table/torso_poses",['home','front','back','left','right'])

                with self:
			self.userdata.object_names = object_names
                        self.add('DETECT_OBJECT_TABLE',skill_state_detectobjectsfront.skill_state_detectobjectsfront(object_names=self.userdata.object_names),
                                transitions={'not_detected':'ended',
                                        'failed':'ended',
					'detected':'ended'})

	def pre_conditions(self):
		
		print "Some preconditions"

	def post_conditions(self):
		print "Some postconditions"

	@property    
	def inputs(self):
		return "Some Input"
    
	@property
	def outputs(self):
		return "Some Output"

	@property
	def requirements(self):
		return "Some Requirements"




if __name__=='__main__':
        rospy.init_node('detect_object_frontside')
        sm = SkillImplementation()
        sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
        sis.start()
        outcome = sm.execute()
        rospy.spin()
