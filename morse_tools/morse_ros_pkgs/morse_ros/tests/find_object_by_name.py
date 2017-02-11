#!/usr/bin/env python

import sys
import rospy
import math

from morse_msgs.srv import FindObject
from std_msgs.msg import String

robot_namespace_ = "/pepito/eye/"

def find_object(Name):
    service_name = robot_namespace_ + 'find_object'
    rospy.wait_for_service(service_name)
    try:
        service_function = rospy.ServiceProxy(service_name, FindObject)
        ret = service_function(Name)
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
    
    object_name = String()
    object_name.data = "videotape_grey_1"
    
    print "object %s, result: %s" % (object_name, find_object(object_name))
