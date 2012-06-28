#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import random
import math
import numpy
import sys
import copy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import *
from cob_srvs.srv import *
from cob_generic_states_experimental.srv import *
from sensor_msgs.msg import LaserScan
# from cob_generic_states_experimental.srv import *

pub_marker = rospy.Publisher("marker2", MarkerArray)

# global variables
min_opening_width = 0.9
current_state = False
min_angle = -3.141 / 2.4
max_angle = 3.141 / 2.4
angle_increment = 0.00872664619237
distance_thresh = 3. # everything further 3m considered open
min_dist = 1.9
delta_dist = 0.2

angles = list(numpy.arange(-2.35619449615, 2.35619449615, angle_increment))
min_index = min(angles.index(x) for x in angles if x >= min_angle)
max_index = max(angles.index(x) for x in angles if x <= max_angle)

mid_poses = []
door_poses = []

def polar2cart(d, alpha):
    return( [d * math.cos(alpha), d * math.sin(alpha) ] )

def handle_door_state(req):
    global current_state, mid_poses, door_poses
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    # return AddTwoIntsResponse(req.a + req.b)
    
    #res = TriggerResponse()
    #res.success.data = current_state
    #res.error_message.data = 'bla'
    
    #res1 = Pose2D()
    #res1.x = 0
    #res1.y = 0
    #res1.theta = 0
    #res = DoorResponse()
    #res.doors = [res1]
    res = DoorResponse()
    res.doors = door_poses  # [ res1, res1 ] # mid_poses
    return res

def scan_callback(data):
    
    global angles,current_state, min_index, max_index, mid_poses, distance_thresh, pub_marker, min_opening_width, door_poses

    #print "scan"
    my_angles = angles[min_index:max_index]
    ranges = list(data.ranges[min_index:max_index])
    #ranges = map(lambda y: y>distance_thresh and distance_thresh or y, ranges)
    ranges = map(lambda y: y>min_dist and distance_thresh or y, ranges)
    cart_points = map(polar2cart, ranges, my_angles)
    #print cart_points

    # opening = [x > distance_thresh for x in ranges]
    start_border_index = []
    stop_border_index = []
    opening_IDs = []
    

    started = False
    for i in range(1, len(ranges)):   # also supposed to work for the start and stop ends
        if math.fabs(ranges[i]-ranges[i-1]) > delta_dist:
            opening_IDs.append(i)
    
    #print opening_IDs
    # print start_border_index
 
    #langles = [None]
    #for i in range(1, len(cart_points)-2):
    #    b = math.sqrt( (cart_points[i+1][0] - cart_points[i-1][0])**2 + (cart_points[i+1][1] - cart_points[i-1][1])**2 )
    #    c = math.sqrt( (cart_points[i][0] - cart_points[i-1][0])**2 + (cart_points[i][1] - cart_points[i-1][1])**2 )
    #    a = math.sqrt( (cart_points[i+1][0] - cart_points[i][0])**2 + (cart_points[i+1][1] - cart_points[i][1])**2 )
    #    langle = math.acos( (a**2 + c**2 - b**2)/(2*a*c) )
    #    langles.append(langle)
    #langles.append(None)
    #print langles
    #print cart_points
    

    mm = MarkerArray()
    mm.markers = []
       
    #marker = Marker()      #  circle of maximal distance (green)
    #marker.header.frame_id = "/base_laser_rear_link"
    #marker.header.stamp = rospy.Time.now()
    #marker.lifetime = rospy.Duration(1.5)
    #marker.ns = "edges"
    #marker.id = 11000
    #marker.type = 2 # SPHERE
    #marker.action = 0 # ADD
    #marker.pose.position.x = 0.
    #marker.pose.position.y = 0.
    #marker.pose.position.z = 0.
    #marker.pose.orientation.x = 0.0
    #marker.pose.orientation.y = 0.0
    #marker.pose.orientation.z = 1.0
    #marker.pose.orientation.w = 1.0
    #marker.scale.x = 2*distance_thresh
    #marker.scale.y = 2*distance_thresh
    #marker.scale.z = 0.25
    #marker.color.a = 0.3
    #marker.color.r = 0. # self.color.r
    #marker.color.g = 1. # self.color.g
    #marker.color.b = 0. # self.color.b
    #mm.markers.append(marker)
    # pub_marker.publish(mm)


    marker = Marker()      #  circle of minimal distance (yellow)
    marker.header.frame_id = "/base_laser_rear_link"
    marker.header.stamp = rospy.Time.now()
    #marker.lifetime = rospy.Duration(1.5)
    marker.ns = "edges"
    marker.id = 8746
    marker.type = 2 # SPHERE
    marker.action = 0 # ADD
    marker.pose.position.x = 0.
    marker.pose.position.y = 0.
    marker.pose.position.z = 0.
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 1.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 2*min_dist
    marker.scale.y = 2*min_dist
    marker.scale.z = 0.30
    marker.color.a = 0.7
    marker.color.r = 1. # self.color.r
    marker.color.g = 1. # self.color.g
    marker.color.b = 0. # self.color.b
    mm.markers.append(marker)
    


    ## special cases: open to right or left
    iStart = []
    iStop = []
    lengths = []
    if ranges[0] >= min_dist:
        ranges[0] = min_dist
        cart_points[0] = polar2cart(ranges[0], my_angles[0])
    if ranges[ len(ranges)-1 ] >= min_dist:
        ranges[len(ranges)-1] = min_dist
        cart_points[len(ranges)-1] = polar2cart(ranges[len(ranges)-1], my_angles[len(ranges)-1])
    started = False
    for i in range(1, len(ranges)):
        if not started and ranges[i] >= min_dist:
            iStart.append(i-1)
            started = True
        elif started and ranges[i] <= min_dist:
            iStop.append(i)
            started = False

    for i in range(0, len(ranges)):     # all ranges (blue):
        marker = Marker()
        marker.lifetime = rospy.Duration(1.5)
        marker.header.frame_id = "/base_laser_rear_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.id = 3000 + i
        marker.type = 2 # SPHERE
        marker.action = 0 # ADD
        marker.pose.position.x = cart_points[i][0]
        marker.pose.position.y = cart_points[i][1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.25
        marker.color.a = 0.7
        marker.color.r = 0.6 # self.color.r
        marker.color.g = 0. # self.color.g
        marker.color.b = 0.8 # self.color.b
        if i==0:
            marker.color.g = 0.
            marker.color.a = 1.0
            marker.color.b = 1.
            marker.scale.x = 0.18
            marker.scale.y = 0.28
            marker.scale.z = 0.3

        mm.markers.append(marker)


    for i in range(0, len(iStart)):
        marker = Marker()
        marker.lifetime = rospy.Duration(1.5)
        marker.header.frame_id = "/base_laser_rear_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.id = 17101+10*i
        marker.type = 2 # SPHERE
        marker.action = 0 # ADD
        marker.pose.position.x = cart_points[iStart[i]][0]
        marker.pose.position.y = cart_points[iStart[i]][1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.7
        marker.color.r = 1. # self.color.r
        marker.color.g = 0.0 # self.color.g
        marker.color.b = 0.0 # self.color.g
        mm.markers.append(marker)

    for i in range(0, len(iStop)):
        marker = Marker()
        marker.lifetime = rospy.Duration(1.5)
        marker.header.frame_id = "/base_laser_rear_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.id = 17102 + 10*i
        marker.type = 2 # SPHERE
        marker.action = 0 # ADD
        marker.pose.position.x = cart_points[iStop[i]][0]
        marker.pose.position.y = cart_points[iStop[i]][1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.7
        marker.color.r = 1.0 # self.color.r
        marker.color.g = 0.0 # self.color.g
        marker.color.b = 1.0 # self.color.g
        mm.markers.append(marker)


    pub_marker.publish(mm)

    print "iStart:"
    print iStart
    print "iStop:"
    print iStop

    for i in range(0, len(iStart)):
        lengths.append( math.sqrt( (cart_points[iStart[i]][0] - cart_points[iStop[i]][0])**2 + (cart_points[iStart[i]][1] - cart_points[iStop[i]][1])**2 ) )
    
    toKeep = []
    for i in range(0, len(iStart)):
        if lengths[i] > min_opening_width:
            toKeep.append(i)
    iStart = [iStart[i] for i in toKeep]
    iStop = [iStop[i] for i in toKeep]
    lengths = [lengths[i] for i in toKeep]


    centerpoints = []
    tangents = []
    normals = []
    phis = []
    for i in range(0, len(iStart)):
        centerpoints.append( [(cart_points[iStart[i]][0] + cart_points[iStop[i]][0])*0.5, (cart_points[iStart[i]][1] + cart_points[iStop[i]][1])*0.5 ]  )
        tangents.append( [ cart_points[iStart[i]][0] - cart_points[iStop[i]][0], cart_points[iStart[i]][1] - cart_points[iStop[i]][1]] )
        normals.append( [ -tangents[len(tangents)-1][1], tangents[len(tangents)-1][0] ] )
        phis.append( math.atan( normals[len(tangents)-1][1] / normals[len(tangents)-1][0] ) )
    door_poses = []
    for i in range(0, len(iStart)):
        p = Pose2D()
        p.x = centerpoints[i][0]
        p.y = centerpoints[i][1]
        p.theta = phis[i]
        door_poses.append(p)

    print len(iStart)
    print door_poses
    for i in range(0, len(iStart)):
        
        
        marker = Marker()
        marker.lifetime = rospy.Duration(1.5)
        marker.header.frame_id = "/base_laser_rear_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.id = 17103 + 10*i
        marker.type = 2 # SPHERE
        marker.action = 0 # ADD
        marker.pose.position.x = centerpoints[i][0]
        marker.pose.position.y = centerpoints[i][1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.7
        marker.color.r = 1.0 # self.color.r
        marker.color.g = 1.0 # self.color.g
        marker.color.b = 0.0 # self.color.g
        mm.markers.append(marker)

        marker = Marker()
        marker.lifetime = rospy.Duration(1.5)
        marker.header.frame_id = "/base_laser_rear_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.id = 17104 + 10*i
        marker.type = 2 # SPHERE
        marker.action = 0 # ADD
        marker.pose.position.x = centerpoints[i][0] + normals[i][0]
        marker.pose.position.y = centerpoints[i][1] + normals[i][1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 0.7
        marker.color.r = 1.0 # self.color.r
        marker.color.g = 0.0 # self.color.g
        marker.color.b = 1.0 # self.color.g
        mm.markers.append(marker)


    pub_marker.publish(mm)

    x = random.randint(1,10)
    if x < 5:
        current_state = False
    else:
        current_state = True
    

def Door_server():
    rospy.init_node('door_server')
    # s = rospy.Service('door_state', Trigger, handle_door_state)
    s = rospy.Service('door_state', Door, handle_door_state)

    # ss = rospy.Subscriber("/scan_front", LaserScan, scan_callback)
    ss = rospy.Subscriber("/scan_rear", LaserScan, scan_callback)
    
    r = rospy.Rate(1) # check once per second
    while not rospy.is_shutdown():  #and t1-t0 <= self.waitDuration and self.doorStatus=="doors_closed":  # and not contact 
        # t1 = rospy.Time.now()
        # contactStatus = rospy.ServiceProxy('/sdh_controller/one_pad_contact', Trigger) # ?correct?
        # contact = contactStatus().success.data
        # print contact      
        #print "hi"

        print "hi"
        #sys.exit(1)
        r.sleep()      

    ss.unregister()
    ## rospy.spin()

if __name__ == "__main__":
    Door_server()

