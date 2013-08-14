
import roslib
import rospy
import math
import random
import sys
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf
import time
import threading

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

class Utils():
  def __init__(self):
    a=0
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


  def goal_approached(self,goal,tl,dist_threshold=0.3):
      (transform_possible,current_position,quaternion)=self.getRobotPose(tl)
      if transform_possible==True:
        [r,p,y]=euler_from_quaternion(quaternion)
        #print r
        #print p
        #print y
        print current_position
        dist_to_goal=self.calc_dist(goal.x,goal.y,current_position[0],current_position[1])
        rospy.loginfo("distance to goal= %f",dist_to_goal)
        if dist_to_goal<=dist_threshold:
          return True
        else:
          return False
      else:
        rospy.logwarn("No transform available - no goal approached detection")
        return False

  def transformPose(self,pose,tl):
    while not rospy.is_shutdown():
        try:
            transformed_pose=tl.transformPose("/map",pose)
            transform_possible=True
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          rospy.sleep(0.5)
          transformed_pose=pose.pose
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
              rospy.sleep(0.5)

    return (transform_possible,current_position,quaternion)



