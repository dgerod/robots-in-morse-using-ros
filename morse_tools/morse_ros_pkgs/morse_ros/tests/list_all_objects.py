#!/usr/bin/env python

import sys
import rospy
import math

from morse_msgs.srv import GetObjects

robot_namespace_ = "/pepito/eye/"

def get_objects():
    service_name = robot_namespace_ + 'get_objects'
    rospy.wait_for_service(service_name)
    try:
        service_function = rospy.ServiceProxy(service_name, GetObjects)
        ret = service_function()
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
    
    print "result: %s" % (get_objects())
