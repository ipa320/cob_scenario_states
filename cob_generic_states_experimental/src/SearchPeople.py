#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')   # todo: additional command line parameter
import rospy
import smach
import smach_ros

from DetectPeople import *
from WonderAround import *

class SearchPeople(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(
            self,
            outcomes=['found','finished','failed'],
            default_outcome='finished',
            #output_keys=['concurrent_stop'],
            outcome_map={'finished':{'WONDER_AROUND':'finished'},
                         'found':{'DETECT_PEOPLE':'found'},
                         'failed':{'DETECT_PEOPLE':'failed'},
                         'failed':{'WONDER_AROUND':'failed'}})
            
        with self:
            smach.Concurrence.add('WONDER_AROUND', WonderAround())
            smach.Concurrence.add('DETECT_PEOPLE', DetectPeople())