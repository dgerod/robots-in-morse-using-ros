#!/usr/bin/env python

import sys
import rospy
import math

from morse_msgs.srv import PlaceJoints
from sensor_msgs.msg import JointState

robot_namespace_ = "/pepito/arm/"

def set_joints(Joints):
    service_name = robot_namespace_ + 'set_joints'
    rospy.wait_for_service(service_name)
    try:
        service_function = rospy.ServiceProxy(service_name, PlaceJoints)
        ret = service_function(Joints)
        return ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
#    if len(sys.argv) == 3:
#       x = int(sys.argv[1])
#       y = int(sys.argv[2])
#   else:
#       print usage()
#       sys.exit(1)
    
    jnts = JointState()
    
    #jnts.position = [math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0];    
    jnts.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    
    print "joints %s, result: %s" % (jnts, set_joints(jnts))
