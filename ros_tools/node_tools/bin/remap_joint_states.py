#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('node_tools')
import rospy
from sensor_msgs.msg import JointState


_publisher = None


def forward_js_topic(data):
    global _publisher
    _publisher.publish(data)


def connection(source, target):
    global _publisher
    rospy.Subscriber(source, JointState, forward_js_topic)
    _publisher = rospy.Publisher(target, JointState, queue_size=10)


def usage():
    return "%s forwards joint_state [source target]" % sys.argv[0]


if __name__ == '__main__':

    try:

        rospy.init_node("remap_joint_states")

        if len(sys.argv) > 3:

            source = str(sys.argv[1:len(sys.argv):2][0])
            target = str(sys.argv[2:len(sys.argv):2][0])

            rospy.loginfo('source   : %s', source)
            rospy.loginfo('target : %s', target)

            connection(source, target)
            rospy.spin()

        else:
           rospy.loginfo(usage())
           rospy.loginfo(sys.argv)
           sys.exit(1)

    except rospy.ROSInterruptException:
        pass

