#!/usr/bin/python
###
#Robot moves to position in room, rotates around the Z-axis and looks for people.
#It stops, when a specific person is found.
#
#Script needs in case of application in simulation:
#  - roscore
#  - kinect driver
#  - cob_people_detection
#  - cob_bringup_sim
#  - cob_navigation_global/2dnav_ros_dwa
#Set name to look for in SetName and Position in SetNavigationGoal.


import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import random
from cob_people_detection_msgs.msg import *
from ApproachPose import *
import sys
from tf import TransformListener
from tf.transformations import euler_from_quaternion

class SelectNavigationGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
      outcomes=['selected','not_selected','failed'],
      output_keys=['base_pose'])
    self.goals = []

  def execute(self, userdata):
    #print self.goals
    #userdata.base_pose = self.goals.pop() # takes last element out of list
    x=2
    y=-2
    th=1.5
    pose = []
    pose.append(x) # x
    pose.append(y) # y
    pose.append(th) # th
    userdata.base_pose=pose

    return 'selected'



class SetName(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['set','failed'],
      input_keys=['id'],
      output_keys=['id'])

  def execute(self, userdata):
    userdata.id='Richard'
    #userdata.id='Richard'
    sss.say(["I am looking for %s!"%str(userdata.id)])
    return 'set'

class Rotate(smach.State):
  #class handles the rotation until program is stopped
  def __init__(self):
    self.tf = TransformListener()
    smach.State.__init__(self,
      outcomes=['finished','failed'],
      input_keys=['base_pose','stop_rotating'],
      output_keys=['detected'])
    self.stop_rotating=False
    rospy.Subscriber("/cob_people_detection/detection_tracker/face_position_array",DetectionArray, self.callback)
    self.label="Unknown"

  def callback(self,msg):
    if len(msg.detections) >0:
        self.label=msg.detections[0].label
        self.stop_rotating=True
    else:
        a=1
    return

  def execute(self, userdata):
    sss.say(["I am going to look around now"])

    if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
        t = self.tf.getLatestCommonTime("/base_link", "/map")
        position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
	[r,p,y]=euler_from_quaternion(quaternion)
	print r
	print p
	print y
        print position
    else:
        print "No transform available"
        return "failed"

    
    time.sleep(1)
    self.stop_rotating=False
    curr_pose=list()
    curr_pose.append(0)
    curr_pose.append(0)
    curr_pose.append(0.1)

    while not rospy.is_shutdown() and self.stop_rotating==False and curr_pose[2]< 3.14:
      handle_base = sss.move_base_rel("base", curr_pose)
      print "rotation command sent ..."
      time.sleep(2)
    print "-->stop rotating"
    userdata.detected=self.label
    return 'finished'

class Talk(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['found','not_found','failed'],
      input_keys=['id','detected']
      )

    self.phrases=[
        "I am looking for ",
        "Hello, nice to meet you! I don't know you yet!",
        ", nice to see you again",
        "No,you are not "
        ]

  def execute(self, userdata):
    name= str(userdata.id)
    print "wanted: %s"%name
    print "found:  %s"% userdata.detected
    if userdata.id != userdata.detected:
      print name
      #speech=self.phrases[3]+name+" !"
      sss.say(['No, you are not %s'%str(name)])
      return 'not_found'
    else:
      sss.say(['I have found you, %s !'%str(name)])
      time.sleep(2)
      return 'found'

class Seek(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'])
        with self:
            smach.StateMachine.add('SETNAME',SetName(),
                                   transitions={'set':'ROTATE',
                                                'failed':'failed'})
            smach.StateMachine.add('SELECT_GOAL',SelectNavigationGoal(),
                                   transitions={'selected':'MOVE_BASE',
                                                'not_selected':'finished',
                                                'failed':'failed'})
            smach.StateMachine.add('MOVE_BASE',ApproachPose(),
                                   transitions={'reached':'ROTATE',
                                                'not_reached':'failed',
                                                'failed':'failed'})
            smach.StateMachine.add('ROTATE',Rotate(),
                                   transitions={'finished':'TALK',
                                                'failed':'failed'})
            smach.StateMachine.add('TALK',Talk(),
                                   transitions={'found':'finished',
                                                #'not_found':'failed',
                                                'not_found':'ROTATE',
                                                'failed':'failed'})

            

            #smach.StateMachine.add('DETECT',DetectObjectsFrontside(['milk','pringles'],mode="one"),
            #                       transitions={'detected':'ANNOUNCE',
            #                                    'not_detected':'ANNOUNCE',
            #                                    'failed':'failed'})

            #smach.StateMachine.add('ANNOUNCE',AnnounceFoundObjects(),
            #                       transitions={'announced':'SELECT_GOAL',
            #                                    'not_announced':'SELECT_GOAL',
            #                                    'failed':'failed'})


















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Seek(),
                transitions={'finished':'ended',
                      'failed':'ended'})

if __name__=='__main__':
  rospy.init_node('SEEK')
  sm = SM()
  sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
  sis.start()
  outcome = sm.execute()
  rospy.spin()
  sis.stop()
