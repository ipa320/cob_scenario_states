#!/usr/bin/env python

# TEMPORARY FUNCTION FOR TESTING THE GRASP SKILL

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros

import temp_skill_sm_detectandgrasp
from abc_skill import SkillsBase

class SkillImplementation(SkillsBase):

    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['success', 'failed'])

        with self:
            self.add('DETECTANDGRASP_SKILL',temp_skill_sm_detectandgrasp.skill_detectandgrasp(),
                transitions={'success':'DETECTANDGRASP_SKILL'})

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
    rospy.init_node('Detectandgrasp')
    sm = SkillImplementation()
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
