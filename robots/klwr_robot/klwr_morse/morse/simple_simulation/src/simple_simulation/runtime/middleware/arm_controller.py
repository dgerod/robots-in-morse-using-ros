import logging; logger = logging.getLogger("morse."+ __name__)

import rospy
from control_msgs.msg import FollowJointTrajectoryAction

from morse_helpers import adapters
from morse.core.services import interruptible
from morse.core.exceptions import MorseServiceError
from morse_comms.srv import PlaceJoints


class ArmControllerForKLWR(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):

        super().__init__(overlaid_object)
        joints = list(overlaid_object.local_data.keys())

        self.namespace = namespace
        name = adapters.morse_to_ros_namespace( self.name() )

        rospy.set_param(name + "/joint_names", joints)

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    @adapters.ros_service(type=PlaceJoints, name="move_to_position")
    def _move_to_position(self, request):

        joints = request

        names = self.overlaid_object.get_joints()
        positions = joints.position
        
        for jdx in range(len(names)):
            channel, is_prismatic = self.overlaid_object._get_joint(names[jdx])            
            if is_prismatic == True:
                self.overlaid_object.set_translation(names[jdx], positions[jdx])
            else:
                self.overlaid_object.set_rotation(names[jdx], positions[jdx]) 
        
        return True
            
    @interruptible
    @adapters.ros_action(type=FollowJointTrajectoryAction, name="follow_joint_trajectory")
    def _follow_joint_trajectory(self, request):

        trajectory_data = request.trajectory

        joint_names = trajectory_data.joint_names
        logger.info(trajectory_data.joint_names)
                
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
        for p in trajectory_data.points:

            point = {}

            pos = dict(zip(joint_names, p.positions))
            point["positions"] = [pos[j] for j in target_joints if j in pos]
            
            vel = dict(zip(joint_names, p.velocities))
            point["velocities"] = [vel[j] for j in target_joints if j in vel]

            acc = dict(zip(joint_names, p.accelerations))
            point["accelerations"] = [acc[j] for j in target_joints if j in acc]

            point["time_from_start"] = self._stamp_to_secs(p.time_from_start)
            points.append(point)

        trajectory = {"starttime": self._stamp_to_secs(trajectory_data.header.stamp),
                      "points": points }
        logger.info(trajectory)
        
        self.overlaid_object.trajectory(
                self.chain_callback(self._follow_joint_trajectory_result), trajectory)

    def _follow_joint_trajectory_result(self, result):
        return result
    
