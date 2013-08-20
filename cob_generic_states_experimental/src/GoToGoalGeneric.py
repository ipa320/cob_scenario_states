#!/usr/bin/python


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

from GoToUtils import *
from cob_map_accessibility_analysis.srv import CheckPerimeterAccessibility
from cob_map_accessibility_analysis.srv import CheckPointAccessibility
############### PARAMETER SETTINGS #######################
#
#global MAP_BOUNDS
#MAP_BOUNDS=[-1.0,4.5,-0.5,1.5] # x_min,x_max,y_min,y_max
#
## threshold defines whether a goal is approached close enough
#global APPROACHED_THRESHOLD
#APPROACHED_THRESHOLD=0.3 #[m]
## topic where person positions are provided, tracked by tha accompany tracker
#global TOPIC_TRACKED_HUMANS
#TOPIC_TRACKED_HUMANS="/accompany/TrackedHumans"
##  set to true if you want to double check if the person is still at the goal
#global DOUBLECHECK
#DOUBLECHECK=True
## threshold defines whether a goal is similar to another goal 
#global SIMILAR_GOAL_THRESHOLD
#SIMILAR_GOAL_THRESHOLD=0.1 #[m]



class GoToGoalGeneric(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['failed','approached_goal_found','approached_goal_not_found','updated_goal'],
        input_keys= ['predefinitions','person_detected_at_goal', 'callback_config','search_while_moving','current_goal','use_perimeter_goal','position_last_seen','person_name'],
        output_keys=['predefinitions','search_while_moving','current_goal','position_last_seen','person_detected_at_goal','person_name'])
    self.detections=list()
    self.callback_activated=False
    self.generic_listener=GenericListener(target_frame="/map")
    self.utils=Utils()
    self.person_detected_at_current_goal=False
    self.update_goal=False
    self.current_goal=False
    self.new_goal=False
    self.current_goal_approached=False
    # flag if perimeter goals are supposed to be approached instead of
    # approaching the goal directly
 #   self.use_perimeter_goal=True --> should be set by userdata for each call individually


  def check_callback(self,name,similar_goal_threshold):
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
        self.update_goal=self.goals_differ(self.new_goal,self.current_goal,similar_goal_threshold)
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
      self.generic_listener.reset()
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
        print "det.label=", det.label
        print "NAME FOUND"
        # when name found  reset detection list
        self.reset()
        return det
    return False


  def goals_differ(self,old_goal,new_goal,threshold=0.2):
    #dist_between_goals=self.utils.calc_dist(old_goal["x"],old_goal["y"],new_goal["x"],new_goal["y"])
    dist_between_goals=self.utils.calc_dist(old_goal.x,old_goal.y,new_goal.x,new_goal.y)
    if dist_between_goals> threshold:
      return True
    else:
      return False

  def get_perimeter_goal(self,goal,radius):

    #  rospy.wait_for_service('map_accessibility_analysis/map_points_accessibility_check',10)
    #  try:
    #    get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_points_accessibility_check', CheckPointAccessibility)
    #    res = get_approach_pose([goal])
    #  except rospy.ServiceException, e:
    #    rospy.logwarn("Service call failed: %s",e)
    #    print "logwarn  returing false"
    #    return False

    #  # check whether goal on perimeter has to be approached
    #  if res.accessibility_flags[0]==False:
      if True:
        rospy.loginfo("Computing goal on perimeter")

        rotational_sampling_step = 10.0/180.0*math.pi
        rospy.wait_for_service('map_accessibility_analysis/map_perimeter_accessibility_check',10)
        try:
          get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_perimeter_accessibility_check', CheckPerimeterAccessibility)
          res = get_approach_pose(goal, radius, rotational_sampling_step)
          valid_poses=res.accessible_poses_on_perimeter
        except rospy.ServiceException, e:
          rospy.logwarn("Service call failed: %s",e)
          print "logwarn  returing false"
          return False
        
        if len(valid_poses) == 0:
            return 0

        # try for a while to get robot pose # TODO check if this is necessary
        for i in xrange(10):
          (trafo_possible,robot_pose,quaternion)=self.utils.getRobotPose(get_transform_listener())
          rospy.sleep(0.2)
          if trafo_possible==True:
            current_pose=Pose2D()
            current_pose.x=robot_pose[0]
            current_pose.y=robot_pose[1]
            break

        if trafo_possible==True:
          closest_pose = Pose2D()
          minimum_distance_squared = 100000.0
          for pose in valid_poses:
            dist_squared = (pose.x-current_pose.x)*(pose.x-current_pose.x)+(pose.y-current_pose.y)*(pose.y-current_pose.y)
            if dist_squared < minimum_distance_squared:
              minimum_distance_squared = dist_squared
              closest_pose = pose
          return closest_pose
        else:
          print "logwarn  returing false"
          rospy.logwarn("Could not get current robot pose - taking first pose in list")
          return valid_poses[0]
        #handle_base = sss.move("base", pose,blocking=block_program)
        #print "commanding move to current goal"

      else:
        rospy.loginfo("Goal accessible")
        return goal

  def command_move(self,goal,block_program=False):
      pose=list()
      pose.append(float(goal.x))
      pose.append(float(goal.y))
      pose.append(float(goal.theta))
      handle_base = sss.move("base", pose,blocking=block_program)
      rospy.loginfo("Commanding move to current goal")


  def execute(self,userdata):
    rospy.loginfo("Person detected at goal: %s",userdata.person_detected_at_goal)
    self.generic_listener.set_config(userdata.callback_config)
    self.activate_callback(reset_detections=True)

    if userdata.predefinitions["double_check"]==True:
      userdata.person_detected_at_goal=False
      self.person_detected_at_current_goal=False
      self.generic_listener.reset()

    movement_unecessary=self.utils.goal_approached(userdata.current_goal,get_transform_listener(),dist_threshold=userdata.predefinitions["approached_threshold"])
    if movement_unecessary==True:
      rospy.loginfo("Move unnecessary - waiting for detections")
      for i in xrange(5):
        self.check_callback(userdata.person_name,userdata.predefinitions["similar_goal_threshold"])
        time.sleep(1)
      if self.person_detected_at_current_goal==True:
          userdata.person_detected_at_goal=True
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
      rospy.loginfo("moving towards goal while searching")

      # activate processing of external information
      # command robot move
      #if self.use_perimeter_goal==True:
      if userdata.use_perimeter_goal==True:
        radius_factor = 1.0
        while radius_factor < 1.5:
          perimeter_goal=self.get_perimeter_goal(userdata.current_goal,radius_factor*userdata.predefinitions["goal_perimeter"])
          if perimeter_goal!=False:
            if perimeter_goal==0:
              radius_factor = radius_factor * 1.2
            else: 
              userdata.current_goal=perimeter_goal
              print "perimeter_goal=", perimeter_goal
              break
          else:
            rospy.loginfo("Commanding move to goal directly, as accessability check is not available")
            break
      self.command_move(userdata.current_goal,block_program=False)

      #TODO check for goal status
      stop_base=False
      while not rospy.is_shutdown() and stop_base==False :
        #check every second for goal updates
        time.sleep(1)
        # when external information makes change of goals necessary
        self.check_callback(userdata.person_name,userdata.predefinitions["similar_goal_threshold"])

        if self.update_goal==True:
          # internal  ---------------
          stop_base=True
          # external ----------------
          userdata.current_goal=self.new_goal
          return 'updated_goal'

        else:
          #TODO somehow get status goal approached



          self.current_goal_approached=self.utils.goal_approached(userdata.current_goal,get_transform_listener(),dist_threshold=userdata.predefinitions["approached_threshold"])
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
      input_keys=[ 'callback_config','person_detected_at_goal','rotate_while_observing','person_name','predefinitions','current_goal','position_last_seen','script_time'],
      output_keys=['callback_config','person_detected_at_goal','rotate_while_observing','person_name','predefinitions','current_goal','position_last_seen','script_time'])
    self.rep_ctr=0
    self.utils=Utils()
    self.generic_listener=GenericListener(target_frame="/map")
    #self.tf = TransformListener()


  def execute(self, userdata):
    self.generic_listener.reset()
    self.rotate_while_observing=True
    self.generic_listener.set_config(userdata.callback_config)

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
      rel_pose.append(-0.1)
      for i in xrange(80):
        if i==5:
          rel_pose.pop()
          rel_pose.append(0.1)
        handle_base = sss.move_base_rel("base", rel_pose,blocking =True)

        # pick detection label which is majority in list
        #TODO pick first is temporary hack
        if userdata.person_name=="NOSET":
          # automatically use firste entry in detections
          (new_name,new_goal)=self.generic_listener.get_pose_for_name()
        else:
          (new_name,new_goal)=self.generic_listener.get_pose_for_name(name=userdata.person_name)

        # check what is returned from generic listener
        if new_goal != False:
            userdata.position_last_seen=new_goal
            userdata.current_goal=new_goal
            userdata.person_detected_at_goal=True
            userdata.use_perimeter_goal = True
            # stop observation
            sss.stop("base")
            return 'detected'
        else:
          userdata.person_detected_at_goal=False
      return 'not_detected'

class SetRandomGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['finished','failed'],
      input_keys= ['predefinitions','current_goal'],
      output_keys=['predefinitions','current_goal'])

  def execute(self, userdata):
      rospy.loginfo("navigating to random goal.")
      map_bounds=userdata.predefinitions["map_bounds"]
      while True:
        new_goal=Pose2D()
        new_goal.x=random.uniform(map_bounds[0],map_bounds[1]) # x
        new_goal.y=random.uniform(map_bounds[2],map_bounds[3]) # y
        new_goal.theta=random.uniform(0,2*3.14) # theta
        goal_poses = [new_goal]
        rospy.wait_for_service('map_accessibility_analysis/map_points_accessibility_check',10)
        try:
          get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_points_accessibility_check', CheckPointAccessibility)
          res = get_approach_pose(goal_poses)
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e
          return 'failed'
        if res.accessibility_flags[0] == True:
          userdata.current_goal=new_goal
          userdata.use_perimeter_goal = False
          break

      print '-------> moving to random goal ', userdata.current_goal
      return 'finished'

class GenericListener():
  def __init__(self,target_frame=None):
    # get configuration from input or default one
    self.config=None
    if target_frame==None:
        target_frame="/map"
    # initialize variables
    self.detections=list()

    self.target_frame=target_frame
    self.utils=Utils()


  def set_config(self,config):
    self.config=config
    rospy.loginfo("Subscribing to %s",config["topicname"])

    rospy.Subscriber(self.config["topicname"],self.config["msgclass"], self.listen)

  def reset(self):
    del self.detections[:]

  def listen(self,msg):
    det_content=getattr(msg,self.config["msg_element"])
    for d in det_content:
      d_content=dir(d)

      # check if config fits incoming message
      if self.config["argname_label"][0]  not in d_content:
        rospy.logerr("Cannot subscribe to topic because argname_label was not found in incoming message")
      elif self.config["argname_position"][0] not in d_content:
        rospy.logerr("Cannot subscribe to topic because argname_position[0] was not found in incoming message")


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
        for n,p in self.detections:
          if name == str(n):
            det_pose=Pose2D()
            det_pose.x=p.x
            det_pose.y=p.y
            det_pose.theta=0
            self.reset()
            return (n,det_pose)
        #extract pose for name return false if not present
    return (name,False)

class SearchPersonGeneric(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'],
                input_keys= ['predefinitions','callback_config','person_name','position_last_seen','current_goal','person_detected_at_goal','position_last_seen'],
                output_keys=['predefinitions','callback_config','person_name','position_last_seen','current_goal','person_detected_at_goal','position_last_seen'])

        with self:
            smach.StateMachine.add("GO",GoToGoalGeneric(),
                                    transitions={'failed':'failed',
                                                  'approached_goal_found':'finished',
                                                  'approached_goal_not_found':'ROTATE',
                                                  'updated_goal':'GO'})
            smach.StateMachine.add("ROTATE",ObserveGeneric(),
                                    transitions={'failed':'failed',
                                                  'detected':'GO',
                                                  'not_detected':'RANDOMGOAL'})
            smach.StateMachine.add("RANDOMGOAL",SetRandomGoal(),
                                    transitions={'failed':'failed',
                                                  'finished':'GO'})
