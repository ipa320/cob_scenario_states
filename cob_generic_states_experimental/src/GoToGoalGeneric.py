#!/usr/bin/python


#TODO transformation from coordinate fmrame of camera to map necessary
# doublecheck.....

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
from geometry_msgs.msg import Pose2D

############### PARAMETER SETTINGS #######################
# threshold defines whether a goal is approached close enough
global APPROACHED_THRESHOLD
APPROACHED_THRESHOLD=0.3 #[m]
# topic where person positions are provided, tracked by tha accompany tracker
global TOPIC_TRACKED_HUMANS
TOPIC_TRACKED_HUMANS="/accompany/TrackedHumans"
#  set to true if you want to double check if the person is still at the goal
global DOUBLECHECK
DOUBLECHECK=True
# threshold defines whether a goal is similar to another goal 
global SIMILAR_GOAL_THRESHOLD
SIMILAR_GOAL_THRESHOLD=0.1 #[m]

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

class GoToGoalGeneric(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['failed','approached_goal_found','approached_goal_not_found','updated_goal'],
        input_keys=[ 'callback_config','search_while_moving','current_goal','position_last_seen','person_name'],
        output_keys=['search_while_moving','current_goal','position_last_seen','person_detected_at_goal','person_name'])
    self.detections=list()
    self.callback_activated=False
    self.generic_listener=GenericListener(target_frame="/map")
    self.utils=Utils()
    self.person_detected_at_current_goal=False
    self.update_goal=False
    self.current_goal=False
    self.new_goal=False
    self.current_goal_approached=False


  def check_callback(self,name):
    # This function is supposed to do the following:
    # - listen to topic
    # - when callback is activated:
    # - extract part of mesage with right label
    # - extract position/pose and calculate it with respect to /map
    # - provide it in uniform formatting regardless of input topic

    #listening to generic topic

    if self.callback_activated == True:
      # get pose for name
      (name,self.new_goal)=self.generic_listener.get_pose_for_name(name=name)
      #print "I got this from a generic callback"
      #print name
      #print self.new_goal
      #print "------------------"
      if self.new_goal!=False:
        self.update_goal=self.goals_differ(self.new_goal,self.current_goal,SIMILAR_GOAL_THRESHOLD)
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
    #dist_between_goals=self.utils.calc_dist(old_goal["x"],old_goal["y"],new_goal["x"],new_goal["y"])
    dist_between_goals=self.utils.calc_dist(old_goal.x,old_goal.y,new_goal.x,new_goal.y)
    if dist_between_goals> threshold:
      return True
    else:
      return False


  def command_move(self,goal,block_program=False):
      #TODO replace the following sss.move() command with:
      #TODO check whether goal is blocked
      #TODO call service that moves base to there or to circle around goal - facing the center
      pose=list()
      pose.append(float(goal.x))
      pose.append(float(goal.y))
      pose.append(float(goal.theta))
      handle_base = sss.move("base", pose,blocking=block_program)
      print "commanding move to current goal"


  def execute(self,userdata):
    self.generic_listener.config=userdata.callback_config
    self.activate_callback(reset_detections=True)

    if DOUBLECHECK==True:
      userdata.person_detected_at_goal=False
      self.person_detected_at_current_goal=False
      self.generic_listener.reset()

    movement_unecessary=self.utils.goal_approached(userdata.current_goal,get_transform_listener())
    if movement_unecessary==True:
      print "MOVE UNNECESSARY waiting for detections"
      for i in xrange(5):
        self.check_callback(userdata.person_name)
        time.sleep(1)
      if self.person_detected_at_current_goal==True:
          return 'approached_goal_found'
      else:
          return 'approached_goal_not_found'



    # set generic listener to  userdata config

    # reset update goal
    self.update_goal=False
    self.current_goal=userdata.current_goal
    # TODO is this still necessary - as whole state is based on the assumption
    # do not search for external position hints
    #if userdata.search_while_moving==False:
    #  print "moving towards goal without searching"
    #  stop_base=False
    #  self.command_move(userdata.current_goal)

    #  while not rospy.is_shutdown() and stop_base==False :
    #    # check if goal has been approached close enough
    #    #if self.utils.goal_approached(userdata.current_goal,get_transform_listener()):
    #    #TODO get goal status when it is approached
    #      sss.stop("base")
    #      stop_base=True
    #      rospy.sleep(0.3) 

    #  #TODO verify the following line
    #  if DOUBLECHECK==True:
    #    userdata.person_detected_at_goal=False
    #  return 'approached_goal'


    #elif userdata.search_while_moving==True:
    if True:
      print "moving towards goal while searching"

      # activate processing of external information
      # command robot move
      self.command_move(userdata.current_goal,block_program=False)

      #TODO check for goal status
      stop_base=False
      print "spinning ..."
      while not rospy.is_shutdown() and stop_base==False :
        #check every second for goal updates
        time.sleep(1)
        # when external information makes change of goals necessary
        self.check_callback(userdata.person_name)

        if self.update_goal==True:
          # internal  ---------------
          stop_base=True
          # external ----------------
          userdata.current_goal=self.new_goal
          return 'updated_goal'

        else:
          #TODO somehow get status goal approached



          self.current_goal_approached=self.utils.goal_approached(userdata.current_goal,get_transform_listener())
          print self.current_goal_approached
          # when goal has been approached and person was detected in the
          # process
          if self.current_goal_approached==True and self.person_detected_at_current_goal==True:
            # internal flags------
            stop_base=True
            # external flags------
            userdata.person_detected_at_goal=True
            sss.stop("base")
            return 'approached_goal_found'

          # when goal has been approached but person was NOT detected in the
          # process
          elif self.current_goal_approached==True and self.person_detected_at_current_goal==False:
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

class ObserveGeneric(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['detected','not_detected','failed'],
      input_keys=[ 'callback_config','person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'],
      output_keys=['callback_config','person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'])
    self.rep_ctr=0
    self.utils=Utils()
    self.generic_listener=GenericListener(target_frame="/map")
    #self.tf = TransformListener()


  def execute(self, userdata):
    #TODO temp
    if DOUBLECHECK==True:
      userdata.person_detected_at_goal=False
    self.generic_listener.reset()
    self.rotate_while_observing=True
    self.generic_listener.config=userdata.callback_config

    # TODO is this still necessary - as whole state is based on the assumption
    #if userdata.rotate_while_observing==False:
    #  # give person time to order and observe in the meantime
    #  rospy.sleep(2)

    #  # pick detection label which is majority in list
    #  #TODO pick first is temporary hack

    #  if userdata.person_name=="NOSET":
    #    # automatically use firste entry in detections
    #    (new_name,new_goal)=self.generic_listener.get_pose_for_name()
    #  else:
    #    (new_name,new_goal)=self.generic_listener.get_pose_for_name(name=userdata.person_name)

    #  # check what is returned from generic listener
    #  if new_goal !=False:
    #      userdata.position_last_seen=new_goal
    #      userdata.person_detected_at_goal=True
    #      return 'detected'
    #  else:
    #    userdata.person_detected_at_goal=False
    #    return 'not_detected'

    #elif userdata.rotate_while_observing==True:
    if True:
      rel_pose=list()
      rel_pose.append(0)
      rel_pose.append(0)
      rel_pose.append(0.1)
      for i in xrange(20):
        handle_base = sss.move_base_rel("base", rel_pose,blocking =True)

        # pick detection label which is majority in list
        #TODO pick first is temporary hack
        if userdata.person_name=="NOSET":
          # automatically use firste entry in detections
          (new_name,new_goal)=self.generic_listener.get_pose_for_name()
        else:
          (new_name,new_goal)=self.generic_listener.get_pose_for_name(name=userdata.person_name)

        # check what is returned from generic listener
        if new_goal !=False:
            userdata.position_last_seen=new_goal
            userdata.current_goal=new_goal
            userdata.person_detected_at_goal=True
            # stop observation
            sss.stop("base")
            return 'detected'
        else:
          userdata.person_detected_at_goal=False
      return 'not_detected'


class GenericListener():
  def __init__(self,target_frame=None,config=None):
    # get configuration from input or default one
    if config==None:
      self.config={"argname_frame":["location","header","frame_id"],
                   "argname_label":["identity"],
                   "argname_position":["location","point"],
                   "argname_header":["location","header"],
                   "topicname":TOPIC_TRACKED_HUMANS,
                   "msgclass":TrackedHumans}
    else:
      self.config=config

    # initialize variables
    self.detections=list()
    #TODO get this to work


    rospy.Subscriber(self.config["topicname"],self.config["msgclass"], self.listen)
    self.target_frame=target_frame



  def reset(self):
    del self.detections[:]

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
        position=self.extract_from_msg(d,self.config["argname_position"])
        name=self.extract_from_msg(d,self.config["argname_label"])
        header=self.extract_from_msg(d,self.config["argname_header"])
        frame=self.extract_from_msg(d,self.config["argname_frame"])



        #print "-----------------------------------------------------"
        #print "-----------------------------------------------------"
        #print name
        #print "-----------------------------------------------------"
        #print position
        #print "-----------------------------------------------------"
        #print frame
        #print "-----------------------------------------------------"
        #print "-----------------------------------------------------"

        # turn position to pose
        pose=PoseStamped()
        pose.header=header
        pose.pose.position=position

        # if necessary transform pose
        if frame !=self.target_frame:
          t_pose=self.utils.transformPose(pose,get_transform_listener())
          #print "trafo necessary"
        else:
          #print "trafo not necessary"
          t_pose=pose

        self.detections.append((name,t_pose.pose.position))


  def extract_from_msg(self,d,keyword_list):
        if len(keyword_list)>1:
          att_it=d
          for syl in keyword_list:
            att_it=getattr(att_it,str(syl))
          val=att_it
        else:
          val=getattr(d,str(keyword_list[0]))
        return val

  def get_pose_for_name(self,name=None):
    if len(self.detections)>0:
      #print "detections available"
      if name==None:
          #extract position for the first name in detections
          det_name=self.detections[0][0]
          det_pose=Pose2D()
          det_pose.x=self.detections[0][1].x
          det_pose.y=self.detections[0][1].y
          det_pose.theta=self.detections[0][1].theta
          return (det_name,det_pose)
      # scan detections for name and pose
      else:
        print "Searching detections for %s"%name
        for n,p in self.detections:
          if name == str(n):
            det_pose=Pose2D()
            det_pose.x=p.x
            det_pose.y=p.y
            det_pose.theta=0
            return (n,det_pose)
        #extract pose for name return false if not present
    return (name,False)

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
        dist_to_goal=self.calc_dist(goal.x,goal.y,current_position[0],current_position[1])
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



class SearchPersonGeneric(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'],
                input_keys=['callback_config','person_name','position_last_seen','current_goal','person_detected_at_goal','position_last_seen'],
                output_keys=['callback_config','person_name','position_last_seen','current_goal','person_detected_at_goal','position_last_seen'])

        with self:
            smach.StateMachine.add("GO",GoToGoalGeneric(),
                                    transitions={'failed':'failed',
                                                  'approached_goal_found':'finished',
                                                  'approached_goal_not_found':'ROTATE',
                                                  'updated_goal':'GO'})
            smach.StateMachine.add("ROTATE",ObserveGeneric(),
                                    transitions={'failed':'failed',
                                                  'detected':'GO',
                                                  'not_detected':'finished'})
