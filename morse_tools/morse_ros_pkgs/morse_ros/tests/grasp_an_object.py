#!/usr/bin/env python

import sys
import rospy
import math

from std_srvs.srv import Trigger

robot_namespace_ = "/pepito/gripper/"

def grab_object():
    service_name = robot_namespace_ + 'close'
    rospy.wait_for_service(service_name)
    try:
        service_function = rospy.ServiceProxy(service_name, Trigger)
        success = service_function()
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def release_object():
    service_name = robot_namespace_ + 'open'
    rospy.wait_for_service(service_name)
    try:
        service_function = rospy.ServiceProxy(service_name, Trigger)
        success = service_function()
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    
    print "result: %s" % (grab_object())
    