#!/usr/bin/python

### State: WaitForOpenElevatorDoors(.py)
### Description:
###   This state assumes that there are two doors in front
###   of the robot, one to the left and one to the right
###   (as at the IPA elevator).
###   The state determines the open/closed state of the
###   doors based on laser scan data.
###   If exactly one of the doors is open before the timeout,
###   the state will return the outcome "door_open" and
###   will put the string "elevator_in_left" or
###   "elevator_in_right" in userdata.base_pose.
###   If none of both doors is open before timeout,
###   the outcome is "door_closed".
###   If the required service "/door_state" (e.g. provided
###   by DoorNode.py) is not found, the outcome is "failed".
### Constructor arguments: 
###   waitDuration = timeout in seconds
###   useTeachedPoses = True(default)/False
###     if true, looks for "elevator_in_left" or "elevator_in_right"
###     on parameter server and puts in pre_coded pose in base_pose
###     if false, puts in pose of open door
### Required services:
###   /door_state
### Required Topics: 
### Parameters:
###   base_poses: elevator_in_left, elevator_in_right
### Outcomes:
###   door_open
###   door_closed
###   failed
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from cob_generic_states_experimental.srv import Door

class WaitForOpenElevatorDoors(smach.State):

  def __init__(self, waitDuration, useTeachedPoses=True):
    self.useTeachedPoses = useTeachedPoses
    self.waitDuration = waitDuration
    smach.State.__init__(
      self, 
      outcomes=['door_open','door_closed','failed'],
      output_keys=['base_pose']) 

  def execute(self, userdata):
    if self.useTeachedPoses:
      try:
        self.elevator_in_left = rospy.get_param('/script_server/base/elevator_in_left')
        self.elevator_in_right = rospy.get_param('/script_server/base/elevator_in_right')
      except:
        print 'parameters missing:'
        print '/script_server/base/elevator_in_left'
        print '/script_server/base/elevator_in_right'
        return 'failed'
    try:
      rospy.wait_for_service('/door_state', 2)
    except:
      print '/door_state service not found!'
      return 'failed'

    t0 = rospy.Time.now()
    t1 = t0

    doorOpen = False;
    r = rospy.Rate(2)
    while not rospy.is_shutdown() and t1-t0 <= self.waitDuration and not doorOpen:
      t1 = rospy.Time.now()
      doorFunc = rospy.ServiceProxy('/door_state', Door)
      doorStatus = doorFunc().doors
      if len(doorStatus)==1:
        if doorStatus[0].y > 0.0: # 
          doorOpen = True
          if self.useTeachedPoses:
            userdata.base_pose=self.elevator_in_left
          else:
            userdata.base_pose=[doorStatus[0].x,doorStatus[0].y,doorStatus[0].theta]
        elif doorStatus[0].y < 0.0: # -
          doorOpen = True
          if self.useTeachedPoses:
            userdata.base_pose=self.elevator_in_right
          else:
            userdata.base_pose=[doorStatus[0].x,doorStatus[0].y,doorStatus[0].theta]
      r.sleep()      

    if doorOpen:
      return 'door_open'
    else:
      return 'door_closed'

class WaitForOpenElevatorDoors_TestUserdata(smach.State):
  def __init__(self):
    smach.State.__init__(
      self, 
      outcomes=['ended'],
      input_keys=['base_pose']) 
  def execute(self, userdata):
    print "This is the content of userdata.base_pose:"
    print userdata.base_pose
    return 'ended'

class WaitForOpenElevatorDoors_TestSM(smach.StateMachine):
  def __init__(self):
    smach.StateMachine.__init__(self,outcomes=['ended_happy','ended_sad','ended_failed'])
    with self:
      smach.StateMachine.add('WAIT_FOR_OPEN_DOOR',WaitForOpenElevatorDoors(rospy.Duration(25),False),transitions={'door_open':'TEST_USERDATA','door_closed':'ended_sad','failed':'ended_failed'})
      smach.StateMachine.add('TEST_USERDATA',WaitForOpenElevatorDoors_TestUserdata(),transitions={'ended':'ended_happy'})

def main():
  rospy.init_node('doortest')
  sm = WaitForOpenElevatorDoors_TestSM()
  outcome = sm.execute()
  rospy.spin()

if __name__=='__main__':
  main()
               
