#!/usr/bin/python


#TODO transformation from coordinate fmrame of camera to map necessary

############### PARAMETER SETTINGS #######################
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import math
import smach
import smach_ros
import random
from cob_people_detection_msgs.msg import *
from accompany_uva_msg.msg import *
from ApproachPose import *
import sys
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf
import time
import threading

############### PARAMETER SETTINGS #######################
# topic where person positions are provided, tracked by tha accompany tracker
global TOPIC_TRACKED_HUMANS
TOPIC_TRACKED_HUMANS="/accompany/TrackedHumans"

###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
  global _tl
  with _tl_creation_lock:
    if _tl==None:
      _tl=tf.TransformListener()
    return _tl
#################################################################################

class GoToGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['approached_goal','failed'],
        input_keys=[ 'search_while_moving','current_goal','position_last_seen','person_name'],
        output_keys=['search_while_moving','current_goal','position_last_seen','person_detected_at_goal','person_name'])
    self.detections=list()
    self.callback_activated=False
    self.generic_listener=GenericListener(target_frame="/map")
    self.utils=Utils()
    self.person_detected_at_current_goal=False
    self.update_goal=False
    self.new_goal=False


  def check_callback(self,msg):
    # This function is supposed to do the following:
    # - listen to topic
    # - when callback is activated:
    # - extract part of mesage with right label
    # - extract position/pose and calculate it with respect to /map
    # - provide it in uniform formatting regardless of input topic

    #listening to generic topic

    if callback_activated == True:
      print "callback is active"
      # get pose for name
      (name,self.new_goal)=self.generic_listener.get_pose_for_name(userdata.person_name)
      if self.new_goal!=False:
        self.update_goal=self.goals_differ(self.new_goal,userdata.current_goal,SIMILAR_GOAL_THRESHOLD)
        # if goal is not updated but detection was available - person is
        # detected at current goal - if goal is updated person is NOT detected
        # at current goal
        if self.update_goal==False:
          self.person_detected_at_current_goal=True
        else:
          self.person_detected_at_current_goal=False
    return



  def activate_callback(self,reset_detections=True):

    if reset_detections==True:
      del self.detections[:]
    self.callback_activated=True

    return

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.detections) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.detections)):
        self.detections.append(msg.detections[i])
        #print "appending %s."%msg.detections[i].label
    return


  def check_detections(self,detections,name):
    print "CHECKING DETECTIONS for %s"%name
    if len(detections)==0:
      return False
    for det in detections:
      #print "checking %s"%str(det.label)
      if name == str(det.label):
        print det.label
        print "NAME FOUND"
        # when name found  reset detection list
        del self.detections[:]
        return det
    return False


  def goals_differ(self,old_goal,new_goal,threshold=0.2):
    dist_between_goals=self.utils.calc_dist(old_goal["x"],old_goal["y"],new_goal["x"],new_goal["y"])
    if dist_between_goals> threshold:
      return True
    else:
      return False


  def command_move(self,goal,block_program=False):
      #TODO replace the following sss.move() command with:
      #TODO check whether goal is blocked
      #TODO call service that moves base to there or to circle around goal - facing the center
      pose=list()
      pose.append(float(goal["x"]))
      pose.append(float(goal["y"]))
      pose.append(float(goal["theta"]))
      handle_base = sss.move("base", pose,blocking=block_program)


  def execute(self,userdata):
    # reset update goal
    self.update_goal=False
    # do not search for external position hints
    if userdata.search_while_moving==False:
      print "moving towards goal without searching"
      stop_base=False
      self.command_move(userdata.current_goal)

      while not rospy.is_shutdown() and stop_base==False :
        # check if goal has been approached close enough
        #if self.utils.goal_approached(userdata.current_goal,get_transform_listener()):
        #TODO get goal status when it is approached
          sss.stop("base")
          stop_base=True
          rospy.sleep(0.3) 

      #TODO verify the following line
      if DOUBLECHECK==True:
        userdata.person_detected_at_goal=False
      return 'approached_goal'


    elif userdata.search_while_moving==True:
      print "moving towards goal while searching"
      # try reaching pose
      if DOUBLECHECK==True:
        userdata.person_detected_at_goal=False

      # activate processing of external information
      self.activate_callback(reset_detections=True)
      # command robot move
      self.command_move(userdata.current_goal,block_program=False)

      #TODO check for goal status
      stop_base=False
      while not rospy.is_shutdown() and stop_base==False :
        #check every second for goal updates
        time.sleep(1)
        # when external information makes change of goals necessary
        self.check_callback()
        if self.update_goal==True:
          # internal  ---------------
          stop_base=True
          # external ----------------
          userdata.current_goal=self.new_goal
          return 'updated_goal'

        else:
          #TODO somehow get status goal approached

          # when goal has been approached and person was detected in the
          # process
          if self.goal_approached==True and self.person_detected_at_current_goal==True:
            # internal flags------
            stop_base=True
            # external flags------
            userdata.person_detected_at_goal=True
            sss.stop("base")
            return 'approached_goal_found'

          # when goal has been approached but person was NOT detected in the
          # process
          elif self.goal_approached==True and self.person_detected_at_current_goal==False:
            # internal flags------
            stop_base=True
            # external flags------
            userdata.person_detected_at_goal=False
            sss.stop("base")
            return 'approached_goal_not_found'
#########################################################################
#        detection=self.check_detections(self.detections,userdata.person_name)
#        # check if goal update is necessary
#        if detection is not False:
#
#          transformed_pose=self.utils.transformPose(detection,get_transform_listener())
#          msg_pos=transformed_pose.pose.position
#
#          det_pos={"name":"det","x":msg_pos.x,"y":msg_pos.y,"theta":0.0}
#          # confirm detection of person at goal
#          userdata.person_detected_at_goal=True
#          if self.utils.update_goal(userdata.current_goal,det_pos):
#            print "updating goal"
#            userdata.current_goal=det_pos
#            userdata.position_last_seen=det_pos
#            print "stop base"
#            sss.stop("base")
#            stop_base=True
#            return 'update_goal'
#          else:
#            print "goal similar to current goal"
#
#        # check if goal has been approached close enough
#        if self.utils.goal_approached(userdata.current_goal,get_transform_listener()):
#          sss.stop("base")
#          stop_base=True
#          return 'approached_goal'
#        else:
#         print "not close enough to person"
#
#      return 'failed'
###########################################################################


class GenericListener():
  def __init__(self,target_frame=None,config=None):
    # get configuration from input or default one
    if config==None:
      self.config={"argname_frame":["location","header","frame_id"],
                   "argname_label":["identity"],
                   "argname_position":["location","point"],
                   "argname_header":["location","header"],
                   "topicname":TOPIC_TRACKED_HUMANS,
                   "msgtype":"TrackedHumans"}
    else:
      self.config=config

    # initialize variables
    self.detections=list()
    #TODO get this to work
    rospy.Subscriber(self.config["topicname"],cons(), self.listen)
    self.target_frame=target_frame

  def listen(self,msg):

    msg_content=dir(msg)[-1]
    det_content=getattr(msg,msg_content)
    for d in det_content:
      d_content=dir(d)

      # check if config fits incoming message
      if self.config["argname_label"][0]  not in d_content:
        print "ERROR: cannot subscribe to topic because argname_label was not found in incoming message"
      elif self.config["argname_position"][0] not in d_content:
        print "ERROR: cannot subscribe to topic because argname_position[0] was not found in incoming message"


      else:
        position=extract_from_msg(self.config["argname_position"])
        name=extract_from_msg(self.config["argname_position"])
        header=extract_from_msg(self.config["argname_header"])
        frame=extract_from_msg(self.config["argname_frame"])



        print "-----------------------------------------------------"
        print name
        print position
        print frame
        print "-----------------------------------------------------"

        # turn position to pose
        pose=PoseStamped()
        pose.header=header
        pose.position=position

        # if necessary transform pose
        if frame !=self.target_frame:
          t_pose=self.utils.transformPose(pose,get_transform_listener())
          print "trafo necessary"
        else:
          print "trafo not necessary"
          t_pose=pose

        self.detections.append(name,t_pose.position)


  def extract_from_msg(self,keyword_list):
        if len(keyword_list)>1:
          att_it=d
          for syl in keyword_list:
            att_it=getattr(att_it,str(syl))
          position=att_it
        else:
          val=getattr(d,keyword_list)
        return val

  def get_pose_for_name(self,name=None):
    if len(self.detections)>0:
      if name==None:
          #extract position for the first name in detections
          det_name=self.detections[0][0]
          det_pose=self.detections[0][1]
          return (det_name,det_pose)
      # scan detections for name and pose
      else:
        for n,p in self.detections:
          if name == str(n):
            return (n,p)
        #extract pose for name return false if not present
    return False

class Utils():
  def __init__(self):
    print "instatiating Utils"


  def extract_detection(self,detections,name='NULL'):
    if name is'NULL':
      if(len(detections))>0:
          # TODO return majority of array not first element
          return detections[0]
      else:
        return False
    else:
      print "CHECKING DETECTIONS for %s"%name
      if len(detections)==0:
        return False
      for det in detections:
        #print "checking %s"%str(det.label)
        if name == str(det.label):
          print det.label
          print "NAME FOUND"
          # when name found  reset detection list
          return det
    return False

  def calc_dist(self,x1,y1,x2,y2):
    dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
    return dist


  def goal_approached(self,goal,tl):
      dist_threshold=APPROACHED_THRESHOLD
      (transform_possible,current_position,quaternion)=self.getRobotPose(tl)
      if transform_possible==True:
        [r,p,y]=euler_from_quaternion(quaternion)
        #print r
        #print p
        #print y
        print current_position
        dist_to_goal=self.calc_dist(goal["x"],goal["y"],current_position[0],current_position[1])
        print "distance to goal= %f"%dist_to_goal
        if dist_to_goal<=dist_threshold:
          return True
        else:
          return False
      else:
        print "No transform available - no goal approached detection"
        return False

  def transformPose(self,pose,tl):
    # TODO make this transformation work  - right now detected position is not the right one
    while not rospy.is_shutdown():
        try:
            transformed_pose=tl.transformPose("/map",pose)
            transform_possible=True
            print "trafo to target frame succcesful"
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print "error looking up transformation "
          rospy.sleep(0.5)
          transformed_pose=pose.pose
          break
    #else:
    #  transformed_pose=pose
    return transformed_pose

  def getRobotPose(self,tl):
    transform_possible=False
    current_position=0
    quaternion=0

    if tl.frameExists("/base_footprint") and tl.frameExists("/map"):
        while not rospy.is_shutdown():
            try:
                (
                    current_position, quaternion) = tl.lookupTransform( "/map","/base_footprint",
                                                                rospy.Time(0))
                transform_possible=True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              print "error looking up transformation "
              rospy.sleep(0.5)

    return (transform_possible,current_position,quaternion)



class GoToGoalGeneric(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'])
        with self:
            smache.StateMachine.add("GO",GoToGoal(),
                                    transitions={'failed':'failed',
                                                  'approached_goal_found':'finished',
                                                  'approached_goal_not_found':'ROTATE',
                                                  'updated_goal':'GO'})
