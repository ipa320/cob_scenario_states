#!/usr/bin/python

import rospy
import smach
import smach_ros

from cob_generic_states_experimental.SelectObjectFromTablet import *
from cob_generic_states_experimental.KeepMoving import *

class KeepMovingUntilObjectSelected(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(
            self,
            outcomes=['objectSelected','quit'],
            default_outcome='quit',
            output_keys=['object_name','concurrent_stop'],
            outcome_map={#'objectSelected':{'SelectObjectFromKeyboard':'objectSelected'},
                          'objectSelected':{'SelectObjectFromTablet':'objectSelected'},
                         'quit':{'SelectObjectFromTablet':'quit'}})
            
        with self:
            smach.Concurrence.add('KeepMoving', KeepMoving())
            #smach.Concurrence.add('SelectObjectFromKeyboard',SelectObjectFromKeyboard(), remapping={'object_name':'object_name'})
            #smach.Concurrence.add('SelectObjectFromTablet',SelectObjectFromTablet(), remapping={'object_name':'object_name','concurrent_stop':'concurrent_stop'})
            smach.Concurrence.add('SelectObjectFromTablet',SelectObjectFromTablet())
 
#rospy.init_node('eHealth2012')
#sm = SM()
#outcome = sm.execute()
#rospy.spin()


