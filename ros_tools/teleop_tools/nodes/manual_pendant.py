#!/usr/bin/env python
# ==============================================================================
#
# dgerod@xyz-lab.org.es
# ==============================================================================

import roslib; roslib.load_manifest('teleop_tools')
import rospy;
import rosbag

from threading import Thread, Semaphore
import xml.dom.minidom

import tf
from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg
from nav_msgs.msg import Path as PathMsg 

# ------------------------------------------------------------------------------

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value
                
def callback(data):
    #global _pub    
    rospy.loginfo(rospy.get_caller_id() + " I heard something.")
    #_pub.publish(data)

class TargetConn(object):
    
    def __init__(self):
        self._Sem = None
        self._JointState = None
        self._Pub = None
        self._TfListener = None
        self._Thread = None
                
        self._UserTopic = "/joint_states"
        self._RobotTopic = "/iri_wam/joint_states"        
        self._readDescription('/iri_wam/robot_description')
    
    def _readDescription(self, ParamName):
        description = get_param(ParamName)
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        use_mimic = get_param('use_mimic_tags', True)
        use_small = get_param('use_smallest_joint_limits', True)
        self.zeros = get_param("zeros")

        pub_def_positions = get_param("publish_default_positions", True)
        pub_def_vels = get_param("publish_default_velocities", False)
        pub_def_efforts = get_param("publish_default_efforts", False)

        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if use_small and len(safety_tags)==1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if use_mimic and len(mimic_tags)==1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0
        
                joint = {'min':minval, 'max':maxval, 'zero':zeroval}
                if pub_def_positions:
                    joint['position'] = zeroval
                if pub_def_vels:
                    joint['velocity'] = 0.0
                if pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint
            
    def sendPosition(self, Position):        
        with self._Sem:
            self._JointState = Position
            
    def toolPose(self):
        Pose = None        
        with self._Sem:
            Pose = self._Pose        
        return Pose
        
    def start(self):
        self._Thread = Thread(target=self.loop)
        self._Thread.daemon = True
        
        self._Sem = Semaphore()
        self._JointState = JointStateMsg()
        self._Pub = rospy.Publisher(self._RobotTopic, JointStateMsg, queue_size=10)
        self._TfListener = tf.TransformListener()
        #rospy.Subscriber(self._userTopic, self._robotTopic, callback)
        
        self._Thread.start()
    
    def stop(self):
        pass        

    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz)
        delta = get_param("delta", 0.0)
        
        num_joints = (len(self.free_joints.items()) +
                      len(self.dependent_joints.items()))        
        for i, name in enumerate(self.joint_list):
            self._JointState.name.append(str(name))
            joint = None
        self._JointState.header.stamp = rospy.Time.now()
        self._JointState.position = num_joints * [0.0]

        while not rospy.is_shutdown():
            with self._Sem:
                self._JointState.header.stamp = rospy.Time.now()
                self._Pub.publish(self._JointState)
                
            try:
                if (self._TfListener.frameExists("iri_wam_link_base") and 
                    self._TfListener.frameExists("iri_wam_link_tcp")):
                    (trans, rot) = self._TfListener.lookupTransform('iri_wam_link_base', 
                                                                    'iri_wam_link_tcp', 
                                                                    rospy.Time(0))
                    with self._Sem:                        
                        p = PoseStampedMsg()                        
                        p.header.stamp = rospy.Time.now()
                        p.header.frame_id = "iri_wam_link_tcp"                        
                        p.pose.position.x = trans[0]
                        p.pose.position.y = trans[1]
                        p.pose.position.z = trans[2]
                        p.pose.orientation.x = rot[0]
                        p.pose.orientation.y = rot[1]
                        p.pose.orientation.z = rot[2]
                        p.pose.orientation.w = rot[3]
                        self._Pose = p
                        
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            r.sleep()

# ------------------------------------------------------------------------------

def writePath(bag_name, topic, path):
    bag = rosbag.Bag(bag_name, "w");
    bag.write(topic, path)
    bag.close()
                
class Display(object):
    
    def __init__(self, RobotName):
        self._PathPub = None
        self._PathTopic = "ToolPath"        
        self._ToolPath = None
        
        if RobotName != "":
            self._PathTopic = "/tp/" + RobotName + "/" + self._PathTopic            
        else:
            self._PathTopic = "/tp/" + self._PathTopic        
        
        self._PathPub = rospy.Publisher(self._PathTopic, PathMsg, queue_size=10)
    
    def setToolPath(self, Trajectory):        
        self._ToolPath = self._prepPathFromCartTraj(Trajectory, "iri_wam_link_base")
        self._PathPub.publish(self._ToolPath)
            
    def _prepPathFromCartTraj(self, Trajectory, FrameId):

        ToolPath = PathMsg()        
        ToolPath.header.frame_id = FrameId
        
        scale_factor = 1.
        for p in Trajectory:
            p.pose.position.x = p.pose.position.x*scale_factor
            p.pose.position.y = p.pose.position.y*scale_factor
            p.pose.position.z = p.pose.position.z*scale_factor
            ToolPath.poses.append(p)            

        return ToolPath 
              
class TeachPendant(object):
    
    def __init__(self, TargetConn, RobotName=""):
        
        self._RobotName = RobotName
        self._TrajectoryFile = "trajectory_joints.bag"
        self._Trajectory = Trajectory()
        
        self._TrajectoryTrace = []
        self._Display = Display(RobotName)
        
        self._Conn = TargetConn
        self._Conn.start()       
     
    def playTrajectory(self):        

        #path = Path("/path", "path.bag")
        #self._Display._PathPub.publish(path._Path)
          
        t = 0.1
        for p in self._Trajectory._Positions:
            self.sendPosition(p)
            rospy.rostime.wallsleep(t)      
        
        # Store the complete trajectory 
        path = Path()
        path._Path = self._Display._ToolPath
        path.write("/path",  "path_2.bag")
        
        self._Trajectory.rewind()
        self._TrajectoryTrace = []
        
    def sendPosition(self, Position):     
        
        # Read current pose        
        self._TrajectoryTrace.append(self._Conn.toolPose())
        self._Display.setToolPath(self._TrajectoryTrace)
        
        # Send joints to the robot  
        p = Position
        if self._RobotName != "":
            for i in range(len(Position.name)):
                p.name[i] = self._RobotName + "_" + Position.name[i]                        
        self._Conn.sendPosition(p)       

class Path(object):
    def __init__(self, Topic="", BagName=""):
        self._Path = None
        
        if Topic != "" and BagName != "":            
            bag = rosbag.Bag(BagName, "r")
            for topic, msg, t in bag.read_messages(Topic):        
                self._Path = msg
            bag.close()        

    def write(self, Topic, BagName):
        bag = rosbag.Bag(BagName, "w");
        bag.write(Topic, self._Path)
        bag.close()
            
class Trajectory(object):
    def __init__(self):
        self._Positions = []
        self._CurrPosition = 0
        
    def loadPtpFromFile(self, BagName, Topic):
        self._Positions = []
        bag = rosbag.Bag(BagName, "r")
        for topic, msg, t in bag.read_messages(Topic):        
            self._Positions.append(msg)
        bag.close()        
        self.rewind()
            
    def rewind(self):
        self._CurrPosition = 0
        
    def nextPosition(self):
        if self._CurrPosition < len(self._Positions)-1:        
            self._CurrPosition += 1
            return self._Positions[self._CurrPosition]
        else:
            return None
    
    def previousPosition(self):
        if self._CurrPosition > 0:        
            self._CurrPosition -= 1
        return self._Positions[self._CurrPosition]

# ==============================================================================
