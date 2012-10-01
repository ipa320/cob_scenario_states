#!/usr/bin/env python

# TEMPORARY STATE MACHINE FOR TESTING THE GRASP SKILL

import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import random
from nav_msgs.msg import Odometry

from simple_script_server import *
sss = simple_script_server()

from abc_sm_skill import SkillsSM

import skill_approachpose
import skill_detectobjectsfront
import skill_detectobjectsback
import skill_state_announcefoundobjects
import skill_grasp

class skill_detectandgrasp(SkillsSM):
    

    def mach_detect(self):
        rospy.loginfo("Executing the Detect Skill!")
        mach =  skill_detectobjectsfront.SkillImplementation()
        return mach
    
    def mach_detect_back(self):
        rospy.loginfo("Executing the Detect Skill!")
        mach =  skill_detectobjectsback.SkillImplementation()
        return mach
    
    def __init__(self):
        
        smach.StateMachine.__init__(self,
                                    outcomes=['success', 'failed'])

        self.apose = None

        with self:

            self.add('DETECT_SKILL_BACK',self.mach_detect_back(),
                     transitions={'ended':'ANNOUNCE_SKILL_BACK'})

            self.add('ANNOUNCE_SKILL_BACK',skill_state_announcefoundobjects.skill_state_announcefoundobjects(),
                     transitions={'announced':'GRASP',
                                  'not_announced':'DETECT_SKILL_BACK',
                                  'failed':'failed'})

            self.add('GRASP',skill_grasp.SkillImplementation(),
                     transitions={'grasped':'DETECT_SKILL_BACK',
                                  'not_grasped':'DETECT_SKILL_BACK',
                                  'failed':'failed'})               

