
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
import numpy as np
###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
  global _tl
  with _tl_creation_lock:
    if _tl==None:
      _tl=tf.TransformListener(True, rospy.Duration(40.0))
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
          print "det.label=", det.label
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
        print "current_position=", current_position, "   yaw=", y
        dist_to_goal=self.calc_dist(goal.x,goal.y,current_position[0],current_position[1])
        #angledist
        rho=180/math.pi
        delta=5/rho
        angle_dist=0.0 # (abs(abs(y)-abs(goal.theta))/delta)*0.1
        rospy.loginfo("angle goal = %f",goal.theta)
        rospy.loginfo("angle robot (yaw)= %f",y)
        rospy.loginfo("angle distance = %f",angle_dist)
        rospy.loginfo("distance to goal= %f",dist_to_goal)
        rospy.loginfo("combined distance to goal= %f",(dist_to_goal+angle_dist))
        # test combined distance against threshold
        if (angle_dist+dist_to_goal)<=dist_threshold:
          return True
        else:
          return False
      else:
        rospy.logwarn("No transform available - no goal approached detection")
        return False

  # TODO make transform pose dependent on target frame argument  - so it is not
  # always transformed to map
  def transformPose(self,pose,tl):
    while not rospy.is_shutdown():
        try:
            #t=rospy.Time(0)
            t=pose.header.stamp
            tl.waitForTransform('/map', pose.header.frame_id, t, rospy.Duration(10))
           # (pos,quat) = tl.lookupTransform( "/map",pose.header.frame_id,t)
           # t_matrix=tl.fromTranslationRotation(pos,quat)
           # point=np.matrix(  ((pose.pose.position.x,),(pose.pose.position.y,),(pose.pose.position.z,),(1,) )     )
           # t_point=np.dot(t_matrix,point)
           # print t_matrix
           # print point
           # print t_point

           # transformed_pose=pose
           # transformed_pose.pose.position.x=t_point[0,0]
           # transformed_pose.pose.position.y=t_point[1,0]
           # transformed_pose.pose.position.z=t_point[2,0]
            
            transformed_pose=tl.transformPose("/map",pose)

            transform_possible=True
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          rospy.sleep(0.2)
    return transformed_pose

  def getRobotPose(self,tl):
    transform_possible=False
    current_position=0
    quaternion=0

    if tl.frameExists("/base_link") and tl.frameExists("/map"):
        while not rospy.is_shutdown():
            try:
                t=rospy.Time(0)
                tl.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
                (current_position, quaternion) = tl.lookupTransform( "/map","/base_link", t)
                transform_possible=True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              print"trafo not found"
              rospy.sleep(0.5)

    return (transform_possible,current_position,quaternion)



