
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
import math
import smach
import smach_ros
import random
from cob_people_detection_msgs.msg import *
from ApproachPose import *
import sys
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf
import time



# predefined goals:
# list of goals that are predefined in the map
# current goal:
# coordinates of goal, that is currently being navigated to

class Scheduler(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['e0_success','e1_success','e2_success','e3_success','e4_success','e5_failure','failed'],
      input_keys= ['person_name','person_detected_at_goal','predefined_goals','current_goal','script_time','search_while_moving' ,'rotate_while_observing'],
      output_keys=['person_name','person_detected_at_goal','predefined_goals','current_goal','script_time','search_while_moving','rotate_while_observing'])
    self.e=0
    self.failure_ctr=0
    self.predefined_goals={
          "couch":  {"name":"couch","x":1,"y":1,"theta":0.0},
        "kitchen": {"name":"kitchen","x":0,"y":-1,"theta":0.0},
        "middle":  {"name":"middle","x":-2,"y":1,"theta":0.0}
        }

    # define script script events
    # e=0 ---> set start goal1 command going there without observing
    #
    # e=1 ---> begin observing goal1  without rotating
    #
    # e=2 ---> when detection has been made go to goal2 - when detections has
    # not been made return to e=1 with roating
    #
    # e=3 ---> wait at goal2 then proceed to Search
    #
    # e=4 ---> check if person was found at goal. If not initiate rotation
    # observation
    #



  def execute(self, userdata):
    # event 0#################################
    if self.e ==0:
      #userdata.current_goal=self.predefined_goals["kitchen"]
      userdata.current_goal=self.predefined_goals["couch"]
      userdata.search_while_moving=False
      # increment event
      self.e= 1
      print ">>>>>>>>>>>>>>>>>>>>> event 0 success -> goint to couch"
      return 'e0_success'

    # event 1#################################
    elif self.e ==1:
      userdata.rotate_while_observing=False
      # increment event
      self.e= 2
      print ">>>>>>>>>>>>>>>>>>>>> event 1 success -> observing"
      return 'e1_success'

    # event 2#################################
    elif self.e==2:
      if userdata.person_detected_at_goal==True:
        userdata.current_goal=self.predefined_goals["kitchen"]
        #userdata.current_goal=self.predefined_goals["couch"]
        userdata.search_while_moving=False
        # increment event
        self.e= 3
        print ">>>>>>>>>>>>>>>>>>>>> event 2 success -> going to kitchen"
        return 'e2_success'
      elif userdata.person_detected_at_goal==False:
        #check couter so it doesn't get stuck here without finding anyone
        if self.failure_ctr>=2:
          print "after observation target could still not be found"
          return 'failed'
        userdata.rotate_while_observing= True
        # do not increment event ->repeat step
        print ">>>>>>>>>>>>>>>>>>>>> event 2 failure -> trying with rotation"
        self.failure_ctr+=1
        return 'e1_success'

    # event 3#################################
    elif self.e==3:
      userdata.current_goal=userdata.position_last_seen
      #userdata.current_goal=self.predefined_goals["middle"]
      #userdata.person_name="Caro"
      time.sleep(10)
      userdata.search_while_moving=True
      self.e =4
      print ">>>>>>>>>>>>>>>>>>>>> event 3 success -> done job in kitchen  - searching begins"
      return 'e3_success'
    # event 4#################################
    elif self.e==4:
      if userdata.person_detected_at_goal==True:
        print ">>>>>>>>>>>>>>>>>>>>> event 4 success -> approached position of target person - finished"
        return 'e4_success'
      elif userdata.person_detected_at_goal==False:
        userdata.rotate_while_observing= True
        self.e =5
        print ">>>>>>>>>>>>>>>>>>>>> event 4 failure -> approached position but didnt find target person - rotating observation"
        return 'e1_success'
    # event 5#################################
    elif self.e==5:
      if userdata.person_detected_at_goal==True:
        userdata.search_while_moving=True
        # decrement to event 4
        self.e=4
        print ">>>>>>>>>>>>>>>>>>>>> event 5 success -> going to new target position"
        return 'e3_success'
      elif userdata.person_detected_at_goal==False:
        userdata.search_while_moving=True
        userdata.rotate_while_observing= True
        # decrement to event 4
        self.e=4
        print ">>>>>>>>>>>>>>>>>>>>> event 5 success -> now searching random positions"
        return 'e5_failure'
    else:
      print " FAILURE - not handled event"



class SetStartGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['set','failed'],
      input_keys=['predefined_goals','current_goal','script_time','search_while_moving'],
      output_keys=['predefined_goals','current_goal','script_time','search_while_moving'])

  def execute(self, userdata):
      userdata.predefined_goals={
          "couch":  {"name":"couch","x":3,"y":-3,"theta":0.0},
        "kitchen": {"name":"kitchen","x":1,"y":-0.1,"theta":0.0},
        "middle":  {"name":"middle","x":1.5,"y":-1.5,"theta":0.0}
        }
      userdata.current_goal=userdata.predefined_goals["couch"]
      userdata.script_time=1
      userdata.search_while_moving=False
      return 'set'

class SetName(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['set','failed'],
      input_keys=['id'],
      output_keys=['id'])

  def execute(self, userdata):
    userdata.id=str(PERSON)
    #userdata.id='Richard'
    sss.say(["I am looking for %s!"%str(userdata.id)])
    return 'set'



class SelectNavigationGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['selected','not_selected','failed'],
      input_keys=['current_goal'],
      output_keys=['base_pose'])

  def execute(self, userdata):
    if len(userdata.current_goal) >0:
      print "NAVIGATING TO %s."%userdata.current_goal["name"]
      pose = []
      pose.append(float(userdata.current_goal["x"])) # x
      pose.append(float(userdata.current_goal["y"])) # y
      pose.append(float(userdata.current_goal["theta"])) # theta
      userdata.base_pose=pose
      return 'selected'
    else:
      return 'failed'

class SelectRandomGoal(smach.State):
  def __init__(self):
    #TODO make random position based on map??
    # temporary fix - hard code positions
    smach.State.__init__(self,
      outcomes=['selected','not_selected','failed'],
      input_keys=['current_goal'],
      output_keys=['current_goal'])

  def execute(self, userdata):
      print "NAVIGATING TO RANDOM GOAL."
      map_bounds=[-1,3,0,-2]
      rand_x=random.uniform(map_bounds[0],map_bounds[1]) # x
      rand_y=random.uniform(map_bounds[2],map_bounds[2]) # x
      rand_theta=random.uniform(0,3.14) # theta
      userdata.current_goal={"name":"random","x":rand_x,"y":rand_y,"theta":rand_theta}
      return 'selected'

class FakeState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['set','failed'],
        input_keys=['script_time','current_goal','position_last_seen','person_name'],
        output_keys=['script_time','current_goal','person_name','position_last_seen'])

  def execute(self,userdata):
    pos={"name":"fake","x":1.5,"y":-1.5,"theta":0.0}
    userdata.position_last_seen=pos
    userdata.person_name="Caro"
    userdata.current_goal=pos
    #userdata.script_time=3
    return 'set'

class GoToGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['approached_goal','update_goal','failed'],
        input_keys=[ 'search_while_moving','current_goal','position_last_seen','person_name'],
        output_keys=['search_while_moving','current_goal','position_last_seen','person_detected_at_goal'])
    rospy.Subscriber("/cob_people_detection/detection_tracker/face_position_array",DetectionArray, self.callback)
    self.detections=list()
    self.tf = TransformListener()

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.detections) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.detections)):
        self.detections.append(msg.detections[i])
        print "appending %s."%msg.detections[i].label
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



  def calc_dist(self,x1,y1,x2,y2):
    dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
    return dist

  def update_goal(self,old_goal,new_goal):
    threshold= 1  #[m]
    dist_between_goals=self.calc_dist(old_goal["x"],old_goal["y"],new_goal["x"],new_goal["y"])

    if dist_between_goals> threshold:
      return True
    else:
      return False

  def goal_approached(self,goal):
      dist_threshold=1.0
      # get position from tf
      if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
          while not rospy.is_shutdown():
              try:
                  (
                      current_position, quaternion) = self.tf.lookupTransform("/base_link", "/map",
                                                                  rospy.Time(0))
                  break
              except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "error looking up transformation "
                rospy.sleep(0.5)

          #t=rospy.Time.now()
          ##self.tf.waitForTransform("/base_link", "/map",t,rospy.Duration(0.2))
          ##t = self.tf.getLatestCommonTime("/base_link", "/map")
          #print  "time checked"
          #rospy.sleep(1)
          #current_position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)

          # calculate angles from quaternion
          [r,p,y]=euler_from_quaternion(quaternion)
          #print r
          #print p
          #print y
          print current_position
          #TODO why switch x ad y
          dist_to_goal=self.calc_dist(goal["x"],goal["y"],current_position[1],current_position[0])
          print "distance to goal= %f"%dist_to_goal
          if dist_to_goal<=dist_threshold:
            return True
          else:
            return False
      else:
          print "No transform available - no goal approached detection"
          return False

  def execute(self,userdata):
    if userdata.search_while_moving==False:
      print "moving towards goal without searching"
      stop_base=False

      pose=list()
      pose.append(float(userdata.current_goal["x"]))
      pose.append(float(userdata.current_goal["y"]))
      pose.append(float(userdata.current_goal["theta"]))
      handle_base = sss.move("base", pose,blocking=False)

      while not rospy.is_shutdown() and stop_base==False :
        # check if goal has been approached close enough
        if self.goal_approached(userdata.current_goal):
          sss.stop("base")
          stop_base=True
        rospy.sleep(0.3) 
      userdata.person_detected_at_goal=False
      return 'approached_goal'



    elif userdata.search_while_moving==True:
      print "moving towards goal while searching"
      # try reaching pose
      userdata.person_detected_at_goal=False

      # delete old detections
      del self.detections[:]
      print "Length detections %i"%len(self.detections)
      pose=list()
      pose.append(float(userdata.current_goal["x"]))
      pose.append(float(userdata.current_goal["y"]))
      pose.append(float(userdata.current_goal["theta"]))
      handle_base = sss.move("base", pose,blocking=False)

    ## init variables
    #stopping_time = 0.0
    #announce_time = 0.0
    #freq = 2.0 # Hz
    #yellow = False


      # check for goal status
      stop_base=False
      while not rospy.is_shutdown() and stop_base==False :
        print "looping.."
        time.sleep(1)
        detection=self.check_detections(self.detections,userdata.person_name)
        # check if goal update is necessary
        if detection is not False:

          msg_pos=detection.pose.pose.position
          det_pos={"name":"det","x":msg_pos.x,"y":msg_pos.y,"theta":0.0}
          # confirm detection of person at goal
          userdata.person_detected_at_goal=True
          if self.update_goal(userdata.current_goal,det_pos):
            print "updating goual"
            userdata.current_goal=det_pos
            userdata.position_last_seen=det_pos
            print "stop base"
            sss.stop("base")
            sss.say(["There you are %s. I am coming for you"%userdata.person_name])
            stop_base=True
            return 'update_goal'
          else:
            print "goal similar to current goal"

        # check if goal has been approached close enough
        if self.goal_approached(userdata.current_goal):
          sss.stop("base")
          stop_base=True
          return 'approached_goal'
        else:
         print "not close enough to person"

      return 'failed'

  #	# finished with succeeded
  #	if (handle_base.get_state() == 3):
  #		sss.set_light('green')
  #		return 'reached'
  #	# finished with aborted
  #	elif (handle_base.get_state() == 4):
  #		sss.set_light('green')
  #		sss.stop("base")
  #		return 'not_reached'
  #	# finished with preempted or canceled
  #	elif (handle_base.get_state() == 2) or (handle_base.get_state() == 8):
  #		sss.set_light('green')
  #		sss.stop("base")
  #		return 'not_reached'
  #	# return with error
  #	elif (handle_base.get_error_code() > 0):
  #		print "error_code = " + str(handle_base.get_error_code())
  #		sss.set_light('red')
  #		return 'failed'

  #	# check if the base is moving
  #	loop_rate = rospy.Rate(freq) # hz
  #	if not self.is_moving: # robot stands still			
  #		# increase timers
  #		stopping_time += 1.0/freq
  #		announce_time += 1.0/freq

  #		# abort after timeout is reached
  #		if stopping_time >= self.timeout:
  #			sss.set_light('green')
  #			sss.stop("base")
  #			return 'not_reached'
  #		
  #		# announce warning after every 10 sec
  #		if announce_time >= 10.0:
  #			sss.say([self.warnings[random.randint(0,len(self.warnings)-1)]],False)
  #			announce_time = 0.0

  #		# set light to "thinking" after not moving for 2 sec
  #		if round(stopping_time) >= 2.0:
  #			sss.set_light("blue")
  #			yellow = False
  #	else:
  #		# robot is moving
  #		if not yellow:
  #			sss.set_light("yellow")
  #			yellow = True
    
    # sleep
    #loop_rate.sleep()






class Observe(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['detected','not_detected','failed'],
      input_keys=[ 'person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'],
      output_keys=['person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'])
    rospy.Subscriber("/cob_people_detection/detection_tracker/face_position_array",DetectionArray, self.callback)
    self.detections=      list()
    self.false_detections=list()
    self.rep_ctr=0
    self.utils=Utils()

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.detections) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.detections)):
        self.detections.append(msg.detections[i])
    return

  def execute(self, userdata):
    if userdata.rotate_while_observing==False:
      # give person time to order and observe in the meantime
      rospy.sleep(5)

      # pick detection label which is majority in list
      #TODO pick first is temporary hack

      if len(self.detections)==0:
        userdata.person_detected_at_goal=False
        return 'not_detected'
      else:
        det=self.utils.extract_detection(self.detections)
        userdata.person_name=det.label
        msg_pos=det.pose.pose.position
        det_pos={"name":"3","x":msg_pos.x,"y":msg_pos.y,"theta":0.0}
        userdata.position_last_seen=det_pos
        #userdata.current_goal=userdata.predefined_goals["kitchen"]
        #sss.say(['Thank you for your order, %s.'%str(self.detections[0].label)])
        userdata.person_detected_at_goal=True
        return 'detected'
    elif userdata.rotate_while_observing==True:
      del self.detections[:]
      rel_pose=list()
      rel_pose.append(0)
      rel_pose.append(0)
      rel_pose.append(0.1)
      for i in xrange(20):
        handle_base = sss.move_base_rel("base", rel_pose,blocking =True)
        det=self.utils.extract_detection(self.detections)
        del self.detections[:]
        if det !=False:
            print "person detected while rotating"
            # stop observation
            sss.stop("base")
            # update goal
            msg_pos=det.pose.pose.position
            det_pos={"name":"3","x":msg_pos.x,"y":msg_pos.y,"theta":0.0}
            userdata.position_last_seen=det_pos
            userdata.person_detected_at_goal=True
            return 'detected'
        else:
            print "person not found at goal"
            userdata.person_detected_at_goal=False
            #return 'not_detected'

      return 'not_detected'



class MoveBaseTemp(smach.State):
    def __init__(self):
      smach.State.__init__(self,
        input_keys=['current_goal'],
        outcomes=['reached','failed'])

    def execute(self,userdata):
      target_pose=list()
      if userdata.current_goal['name'] is "kitchen":
        target_pose.append(0.1)
        target_pose.append(0)
        target_pose.append(0)
      elif userdata.current_goal['name'] is "couch":
        target_pose.append(-0.1)
        target_pose.append(0)
        target_pose.append(0)
      else:
        target_pose.append(-0.1)
        target_pose.append(-0.1)
        target_pose.append(0)


      for step in xrange(2):
        handle_base = sss.move_base_rel("base", target_pose)
      return 'reached'



#class Rotate(smach.State):
#  #class handles the rotation until program is stopped
#  def __init__(self):
#    self.tf = TransformListener()
#    smach.State.__init__(self,
#      outcomes=['finished','failed'],
#      input_keys=['base_pose','stop_rotating','person_id'],
#      output_keys=['detected'])
#    rospy.Subscriber("/cob_people_detection/detection_tracker/face_position_array",DetectionArray, self.callback)
#    self.stop_rotating=False
#    self.detections=      list()
#    self.false_detections=list()
#
#  def callback(self,msg):
#    # go through list of detections and append them to detection list
#    if len(msg.detections) >0:
#      #clear detection list
#      del self.detections[:]
#      for i in xrange( len(msg.detections)):
#        self.detections.append(msg.detections[i].label)
#    return
#
#  def execute(self, userdata):
#    print "ACCOMPANY-> ROTATE"
#    sss.say(["I am going to take a look around now."])
#
#    # get position from tf
#    if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
#        t = self.tf.getLatestCommonTime("/base_link", "/map")
#        position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
#  # calculate angles from quaternion
#	[r,p,y]=euler_from_quaternion(quaternion)
#	#print r
#	#print p
#	#print y
#  #print position
#    else:
#        print "No transform available"
#        return "failed"
#
#    time.sleep(1)
#    self.stop_rotating=False
#    # create relative pose - x,y,theta
#    curr_pose=list()
#    curr_pose.append(0)
#    curr_pose.append(0)
#    curr_pose.append(0.1)
#
#    while not rospy.is_shutdown() and self.stop_rotating==False and curr_pose[2]< 3.14:
#      handle_base = sss.move_base_rel("base", curr_pose)
#
#      #check in detection and react appropriately
#      for det in self.detections:
#        # right person is detected
#        if det == userdata.id:
#          self.stop_rotating=True
#          sss.say(['I have found you, %s! Nice to see you.'%str(det)])
#        elif det in self.false_detections:
#        # false person is detected
#          print "Already in false detections"
#       #  person detected is unknown - only react the first time
#        elif det == "Unknown":
#          print "Unknown face detected"
#          sss.say(['Hi! Nice to meet you, but I am still searching for %s.'%str(userdata.id)])
#          self.false_detections.append("Unknown")
#      # wrong face is detected the first time
#        else:
#          self.false_detections.append(det)
#          print "known - wrong face detected"
#          sss.say(['Hello %s! Have you seen %s.'%(str(det),str(userdata.id))])
#      #clear detection list, so it is not checked twice

#      del self.detections[:]
#      time.sleep(2)
#
#    print "-->stop rotating"
#    return 'finished'

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
      #speech=self.phrases[3]+name+" !"
      sss.say(['No, I am sorry, but you are not %s.'%str(name)])
      return 'not_found'
    else:
      sss.say(['I have found you, %s! Nice to see you.'%str(name)])
      time.sleep(2)
      return 'found'

class Seek(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'])
        with self:
  #          smach.StateMachine.add('SETNAME',SetName(),
  #                                 transitions={'set':'SETSTARTGOALNAME',
  #                                              'failed':'failed'})
            smach.StateMachine.add('SCHEDULER',Scheduler(),
                                   transitions={'e0_success':'GOTOGOAL',
                                                'e1_success':'OBSERVE',
                                                'e2_success':'GOTOGOAL',
                                                'e3_success':'GOTOGOAL',
                                                'e4_success':'finished',
                                                'e5_failure':'SELECT_RANDOM_GOAL',
                                                'failed':'failed'})
            #smach.StateMachine.add('FAKE_STATE',FakeState(),
            #                       transitions={'set':'SEARCH',
            #                                    'failed':'failed'})
            #smach.StateMachine.add('SETSTARTGOAL',SetStartGoal(),
            #                       transitions={'set':'GOTOGOAL',
            #                                    'failed':'failed'})
            smach.StateMachine.add('SELECT_RANDOM_GOAL',SelectRandomGoal(),
                                   transitions={'selected':'GOTOGOAL',
                                                'not_selected':'failed',
                                                'failed':'failed'})
           # smach.StateMachine.add('SELECT_GOAL',SelectNavigationGoal(),
           #                        transitions={'selected':'SEARCH',
           #                                     'not_selected':'failed',
           ##                                     'failed':'failed'})
           # smach.StateMachine.add('MOVE_BASE_TEMP',MoveBaseTemp(),
           #                        transitions={'reached':'OBSERVE',
           #                                     'failed':'failed'})
           # smach.StateMachine.add('MOVE_BASE',ApproachPose(),
           #                        transitions={'reached':'OBSERVE',
           #                                     'not_reached':'failed',
           #                                     'failed':'failed'})
            smach.StateMachine.add('OBSERVE',Observe(),
                                   transitions={'failed':'failed',
                                                'detected':'SCHEDULER',
                                                'not_detected':'SCHEDULER'})
           # smach.StateMachine.add('OBSERVE_search',Observe(),
           #                        transitions={'failed':'failed',
           #                                     'detected':'SELECT_GOAL',
           #                                     'not_detected':'SELECT_GOAL',

            smach.StateMachine.add('GOTOGOAL',GoToGoal(),
                                   transitions={'failed':'failed',
                                                'update_goal':'GOTOGOAL',
                                                'approached_goal':'SCHEDULER'})



            #smach.StateMachine.add('ROTATE',Rotate(),
            #                       transitions={'finished':'finished',
            #                                    'failed':'failed'})
            #smach.StateMachine.add('TALK',Talk(),
            #                       transitions={'found':'finished',
            #                                    #'not_found':'failed',
            #                                    'not_found':'ROTATE',
            #                                    'failed':'failed'})




            #smach.StateMachine.add('DETECT',DetectObjectsFrontside(['milk','pringles'],mode="one"),
            #                       transitions={'detected':'ANNOUNCE',
            #                                    'not_detected':'ANNOUNCE',
            #                                    'failed':'failed'})

            #smach.StateMachine.add('ANNOUNCE',AnnounceFoundObjects(),
            #                       transitions={'announced':'SELECT_GOAL',
            #                                    'not_announced':'SELECT_GOAL',
            #                                    'failed':'failed'})



# TODO:
# set in rotate that default position is to be approached
# then transition to SEARCH again with updated position


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



class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Seek(),
                transitions={'finished':'ended',
                      'failed':'ended'})

if __name__=='__main__':
  print" Accompany Robot Demonstration Script"

  print" For this script to work you have to run:"
  print"- Robot/Simulation"
  print"- Navigation"
  print"- Kinect"
  print"- cob_people_detection"
  #global PERSON
  #PERSON=sys.argv[1]
  rospy.init_node('Accompany')
  sm = SM()
  sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
  sis.start()
  outcome = sm.execute()
  rospy.spin()
  sis.stop()
