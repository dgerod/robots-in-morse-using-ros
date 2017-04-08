#!/usr/bin/env python
# ==============================================================================
# 
# ==============================================================================

import sys
import roslib; roslib.load_manifest('node_tools')
import rospy;

import rosbag
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path 

_pub = None

def readCartTrajectory(bag_name):
    trajectory = []    

    bag = rosbag.Bag(bag_name, "r")
    for topic, msg, t in bag.read_messages("/pose"):        
        trajectory.append( msg )
    bag.close()
            
    return trajectory

def writePath(bag_name, path):
    bag = rosbag.Bag(bag_name, "w");
    bag.write("/path", path)
    bag.close()

def connection(talker):
    global _pub    
    _pub = rospy.Publisher(talker, Path, queue_size=10)
    
if __name__ == '__main__':
    
    try:
        rospy.init_node("poses_to_path")
        connection("/path")
       
        poses_bag_file = sys.argv[1]
        path_bag_file = sys.argv[2]
    
        trajectory = readCartTrajectory(poses_bag_file)        
        path = Path()
        
        scale_factor = 1/1000.
        
        path.header.frame_id = "iri_wam_link_base"
        for p in trajectory:
            p.pose.position.x = p.pose.position.x*scale_factor
            p.pose.position.y = p.pose.position.y*scale_factor
            p.pose.position.z = p.pose.position.z*scale_factor
            path.poses.append(p)            
        print "Path len: ", len(trajectory) 
            
        writePath(path_bag_file, path)

        #rospy.rostime.wallsleep(0.5)        
        #_pub.publish(path)
        
    except rospy.ROSInterruptException:
        pass

# ==============================================================================
