import logging; logger = logging.getLogger("morse."+ __name__)
import roslib, rospy

from morse_helpers import adapters
from morse.core.blenderapi import mathutils
from morse.core.services import interruptible
from morse.core.exceptions import MorseServiceError
from morse_msgs.srv import PlaceIkTarget
from morse_msgs.srv import PlaceJoints

from control_msgs.msg import FollowJointTrajectoryAction
from sensor_msgs.msg import JointState

class ArmStatePublisher(adapters.ROSPublisher):
    """ Publishes a JointState topic and set kuka_{1-7} to the position[0-6]. """
    ros_class = JointState

    def default(self, ci='unused'):

        message = JointState()
        message.header = self.get_ros_header()      
        
        message.name = [''] * 7
        message.position = [0] * 7
        message.velocity = [0] * 7
        message.effort = [0] * 7
        
        # Define name used to export joints
        base_name = "kuka_joint_"
        
        for i in range(7):
            message.name[i] = base_name + ("%d" % (i+1) )
            message.position[i] = self.data[ "kuka_" + ("%d" % (i+1) ) ]

        self.publish(message)

class ArmCtrlByActions(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):
        # Call the constructor of the parent class
        super().__init__(overlaid_object)

        joints = list(overlaid_object.local_data.keys())

        self.namespace = namespace
        name = adapters.morse_to_ros_namespace( self.name() )

        # ---
        
        #base_name = "iri_kuka_joint_"
        
        #joints = []
        #for i in range(7):
        #  joint_name = base_name + ("%d" % (i+1) )
        #  joints.append( joint_name )
        
        rospy.set_param(name + "/joint_names", joints)

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def name(self):

        if self.namespace:
            return self.namespace
        else:
            return super().name()

    # ------------------------------------------------------

    @adapters.ros_service(type = PlaceIkTarget)
    def place_by_pose(self, req):  
        
        pose = req        
        translation = [pose.position.x, pose.position.y, pose.position.z]
        orientation = mathutils.Quaternion((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)).to_euler()     
        
        logger.info(orientation)
        
        ik_targets = self.overlaid_object.list_IK_targets()
        
        # TBD-Fix: The 'ik_targets' variable should be pass to next method.
        IK_TARGET = ik_targets[0]
        logger.info(ik_targets)     
        logger.info(IK_TARGET)     
        
        self.overlaid_object.place_IK_target(IK_TARGET, translation, orientation, False)
        
        # TBD-Fix:
        #self.overlaid_object._suspend_ik_targets();
        #self.overlaid_object._ik_targets_destinations = {}
        
        return True
    
    # ------------------------------------------------------
    # The name of the ROS service is based on the name of this function.
     
    # It is not a place as it produce a movement, change code.
    @adapters.ros_service(type = PlaceJoints)
    def set_joints(self, req):
        
        joints = req        
        names = self.overlaid_object.get_joints()
        positions = joints.position
        
        for jdx in range(len(names)):
        
            channel, is_prismatic = self.overlaid_object._get_joint(names[jdx])            
            if is_prismatic == True:
                self.overlaid_object.set_translation(names[jdx], positions[jdx])
            else:
                self.overlaid_object.set_rotation(names[jdx], positions[jdx]) 
        
        return True
            
    # ------------------------------------------------------
    # The name of the ROS action is based on the name of this function.
        
    @interruptible
    @adapters.ros_action(type = FollowJointTrajectoryAction)
    def follow_joint_trajectory(self, req):
        """
        Fill a MORSE trajectory structure from ROS JointTrajectory
        """
        traj = {}
        req = req.trajectory
        
        traj["starttime"] = self._stamp_to_secs(req.header.stamp)

        # Read joint names in message        
        joint_names = req.joint_names
        logger.info(req.joint_names)
                
        # Overwrite joint names from message to match ones defined by MORSE        
        for i in range( len(joint_names) ):
            joint_names[i] = joint_names[i].replace("kuka_joint", "kuka")        
        logger.info(joint_names)
        
        # Read positions from message
        target_joints = self.overlaid_object.local_data.keys()
        logger.info( target_joints )
        
        # Check if trajectory is correct or not
        diff = set(joint_names).difference(set(target_joints))
        if diff:
            raise MorseServiceError("Trajectory contains unknown joints! %s" % diff)

        points = []
        for p in req.points:
            point = {}

            # Re-order joint values to match the local_data order

            pos = dict(zip(joint_names, p.positions))
            point["positions"] = [pos[j] for j in target_joints if j in pos]
            
            vel = dict(zip(joint_names, p.velocities))
            point["velocities"] = [vel[j] for j in target_joints if j in vel]

            acc = dict(zip(joint_names, p.accelerations))
            point["accelerations"] = [acc[j] for j in target_joints if j in acc]

            point["time_from_start"] = self._stamp_to_secs(p.time_from_start)
            points.append(point)

        traj["points"] = points
        logger.info(traj)
        
        self.overlaid_object.trajectory(
                self.chain_callback(self.follow_joint_trajectory_result), traj)

    def follow_joint_trajectory_result(self, result):
        return result
    
    # ------------------------------------------------------
