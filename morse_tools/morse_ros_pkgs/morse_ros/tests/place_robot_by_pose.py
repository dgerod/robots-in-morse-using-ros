#!/usr/bin/env python

import sys
import rospy
import math

from tf import transformations
from morse_msgs.srv import PlaceIkTarget
from geometry_msgs.msg import Pose as PoseMsg

robot_namespace_ = "/pepito/arm/"

def place(Pose):
    service_name = robot_namespace_ + 'place_by_pose'
    rospy.wait_for_service(service_name)
    try:
        service_function = rospy.ServiceProxy(service_name, PlaceIkTarget)
        success = service_function(Pose)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def preparePoseMsg(Pose):
    
    p = Pose[0:3]
    q = transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
    
    Msg = PoseMsg()
    Msg.position.x = p[0]
    Msg.position.y = p[1]
    Msg.position.z = p[2]
    Msg.orientation.x = q[0]
    Msg.orientation.y = q[1]
    Msg.orientation.z = q[2]
    Msg.orientation.w = q[3]
    
    return Msg
    
def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
#    if len(sys.argv) == 3:
#       x = int(sys.argv[1])
#       y = int(sys.argv[2])
#   else:
#       print usage()
#       sys.exit(1)

    # global
    p = [0.0677100345492, -0.15619122982, 0.5]
    q = [0.0, 0.70710682869, -0.0, 0.70710682869]
    
    # relative
    p = [0.16206035018, -0.535947144032, -0.535947144032]
    q = [0.52021330595, -0.511130034924, 0.488041520119, 0.479520320892]
    
    p = [-0.5, 0.25, 0.5]
    q = [0.0, 0.70710682869, -0.0, 0.70710682869]

    p = [0.0, 0.0, 0.0]
    q = [0.0, 1.0, 0.0, 0.0]
    
    pose = PoseMsg()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    print "pose %s, result: %s" % (pose, place(pose))
