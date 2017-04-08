#!/usr/bin/env python
# ==============================================================================
# Send JointState messages from one topic (listener) to another (talker).
# dgerod@xyz-lab.org.es
# ==============================================================================

import roslib; roslib.load_manifest('node_tools')
import rospy;
from sensor_msgs.msg import JointState

_pub = None

def forward_js_topic(data):
    global _pub    
    rospy.loginfo(rospy.get_caller_id() + " I heard something.")
    _pub.publish(data)

def connection(listener, talker):
    global _pub    
    _pub = rospy.Publisher(talker, JointState, queue_size=10)
    rospy.Subscriber(listener, JointState, forward_js_topic)

if __name__ == '__main__':
    try:
        listener = "/joint_states"
        talker = "/iri_wam/joint_states"
    	
        rospy.init_node("remap_joint_states")
        connection(listener, talker)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

# ==============================================================================
