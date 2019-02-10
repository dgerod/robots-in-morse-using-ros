import logging; logger = logging.getLogger("morse."+ __name__)

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback

from morse_helpers import adapters
from morse.core.services import interruptible
from morse.core.exceptions import MorseServiceError
from morse.core import status

from morse_comms.srv import PlaceJoints, PlaceJointsRequest


from zaps_utilities.types.checker import accepts
from zaps_utilities.communication.service import ServiceClient, MultiServiceClient


class MoveToPosition(ServiceClient):
    def __init__(self):
        super().__init__('/adept_viper_s650/', 'move_to_position', PlaceJoints)


class MotionClient(MultiServiceClient):
    def __init__(self):
        super().__init__()
        self._register_client('move', MoveToPosition())

    def move_to_position(self, request):
        return self._call('move', request)


class FollowJointTrajectoryActionProvider(object):

    _result = FollowJointTrajectoryResult()
    _feedback = FollowJointTrajectoryFeedback()

    def __init__(self, controller):

        self._controller = controller

        name = self._controller.name() + "/follow_joint_trajectory"
        self._as = actionlib.SimpleActionServer(name, FollowJointTrajectoryAction, execute_cb=self._execute,
                                                auto_start=False)

        self._as.start()

    def _execute(self, goal):

        #request = PlaceJointsRequest()
        #request.joints.name = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
        #request.joints.position = [1.57,0,0,0,0,0]

        trajectory_data = goal.trajectory
        joints_name = trajectory_data.joint_names

        executor = TrajectoryExecutor(None)
        trajectory = executor.prepare_trajectory(joints_name, trajectory_data)

        for p in self._trajectory["points"]:

            request = PlaceJointsRequest()
            request.joints.name = trajectory.name
            request.joints.position = p["positions"]
            self._controller.move_to_position(request)

        self._as.set_succeeded(FollowJointTrajectoryResult(FollowJointTrajectoryResult.SUCCESSFUL, ""))


class TrajectoryExecutor:

    def __init__(self, overlaid_object):
        self._overlaid_object = overlaid_object
        self._trajectory = None

    def prepare_trajectory(self, target_joints, trajectory_data):

        joint_names = trajectory_data.joint_names
        logger.info(trajectory_data.joint_names)

        # Read positions from message
        #target_joints = self._overlaid_object.local_data.keys()
        #logger.info(target_joints)

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

        self._trajectory = {"starttime": self._stamp_to_secs(trajectory_data.header.stamp),
                      "points": points}

        return self._trajectory

    def go(self):
        for p in self._trajectory["points"]:
            self._move_joints(p["positions"])
        return True

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def _move_joints(self, joints):

        names = self._overlaid_object.get_joints()
        positions = joints
        print(positions)

        for jdx in range(len(names)):
            self._overlaid_object.set_rotation(names[jdx], positions[jdx])


import math
from collections import OrderedDict
from morse.core.morse_time import time_isafter
from morse.core.services import service, async_service, interruptible

class DefaultActionBehavior:

    def __init__(self, overlaid_object):
        self._overlaid_object = overlaid_object

        self._executing_trajectory = False
        self._trajectory = None

        self._position_reached = False

    @async_service
    def set_trajectory(self, trajectory):

        self._overlaid_object._suspend_ik_targets()

        start_time = self._overlaid_object.robot_parent.gettime()
        if 'starttime' in trajectory:
            trajectory["starttime"] = max(start_time, trajectory["starttime"])
        else:
            trajectory["starttime"] = start_time

        self._trajectory = trajectory
        self._executing_trajectory = True

    def _execute_trajectory(self):
        if self._executing_trajectory is False:
            return

        t = self._overlaid_object.robot_parent.gettime()
        trajectory = self._trajectory

        try:
            if time_isafter(trajectory["starttime"], t):
                return

            if time_isafter(t, trajectory["starttime"] + trajectory["points"][-1]["time_from_start"]):

                # trajectory execution is over!
                self._executing_trajectory = False
                self._overlaid_object.completed(status.SUCCESS, None)

            for p in trajectory["points"]:
                end = trajectory["starttime"] + p["time_from_start"]
                if "started" in p and time_isafter(end, t):
                    # currently going to this waypoint: fine!
                    break

                elif "started" not in p and time_isafter(end, t):
                    # start the new waypoint
                    allocated_time = end - t
                    assert (allocated_time > 0)

                    target = OrderedDict(zip(self._overlaid_object.local_data.keys(),
                                             p["positions"]))

                    for joint in target.keys():
                        # compute the distance based on actual current joint pose
                        dist = target[joint] - self._overlaid_object._get_joint_value(joint)
                        self._overlaid_object.joint_speed[joint] = dist / allocated_time

                    self._overlaid_object.local_data = target

                    p["started"] = True
                    break

                elif "started" not in p and time_isafter(t, end):
                    logger.warning("Skipped a waypoint on armature <%s>. Wrong 'time_from_start'?" %
                                   self._overlaid_object.name())

                # case: "started" and t > end: do nothing, go to next point
        except KeyError as ke:
            self._executing_trajectory = False
            self._overlaid_object.completed(status.FAILED, "Error: invalid trajectory: key %s was expected." % ke)

    def _update_armature(self):

        armature = self._overlaid_object.bge_object
        overlaid_object = self._overlaid_object

        self._position_reached = True
        for channel in armature.channels:

            # we assume a joint is prismatic (translation joint) if its IK
            # 'stretch' parameter is non-null
            is_prismatic = self._overlaid_object._is_prismatic(channel)

            joint = channel.name

            if joint in self._overlaid_object.joint_speed and self._overlaid_object.joint_speed[joint]:
                speed = self._overlaid_object.joint_speed[joint]
            else:
                speed = self._overlaid_object.linear_speed if is_prismatic else self._overlaid_object.radial_speed

            # Retrieve the rotation or translation axis
            axis_index = next(i for i, j in enumerate(self._overlaid_object.find_dof(channel)) if j)

            if is_prismatic:
                # we take the last index ('Z') of the pose of the HEAD of the
                # bone as the absolute translation of the joint. Not 100% sure
                # it is right...
                dist = self._overlaid_object.local_data[joint] - channel.pose_head[2]
            else:
                dist = self._overlaid_object.local_data[joint] - channel.joint_rotation[axis_index]

            w = math.copysign(speed / self._overlaid_object.frequency, dist)

            if is_prismatic:
                if not abs(dist) < self._overlaid_object.distance_tolerance:
                    self.position_reached = False

                    trans = channel.location
                    trans[axis_index] += w
                    channel.location = trans
            else:
                if not abs(dist) < self._overlaid_object.angle_tolerance:
                    self.position_reached = False

                    rot = channel.joint_rotation
                    rot[axis_index] += w
                    channel.joint_rotation = rot

        armature.update()
        return joint

    def _check_position_is_reached(self, joint):
        if self._position_reached:
            print("position reached")
            if not self._executing_trajectory:
                self._overlaid_object.completed(status.SUCCESS, None)
            if joint in self._overlaid_object.joint_speed:
                del self._overlaid_object.joint_speed[joint]
        else:
            print("not position reached")

    def default_action(self):
        self._execute_trajectory()
        joint = self._update_armature()
        self._check_position_is_reached(joint)


def armature_default_action(self):
    print("armature_default_action")
    self._my_behavior.default_action()


class ArmControllerForOatViper(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):

        super().__init__(overlaid_object)
        joints = list(overlaid_object.local_data.keys())

        self.namespace = namespace
        name = adapters.morse_to_ros_namespace( self.name())
        rospy.set_param(name + "/joint_names", joints)

        #import types
        #self.overlaid_object._my_behavior = DefaultActionBehavior(self.overlaid_object)
        #self.overlaid_object.default_action = types.MethodType(armature_default_action, self.overlaid_object)

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
            self.overlaid_object.set_rotation(names[jdx], positions[jdx])

        return True

    def execute_follow_joint_trajectory_action(self, request):

        executor = TrajectoryExecutor(self.overlaid_object)
        executor.prepare_trajectory(request.trajectory)
        executor.go()

        #print("Finish")

    @interruptible
    @adapters.ros_action(type=FollowJointTrajectoryAction, name="follow_joint_trajectory")
    def _follow_joint_trajectory(self, request):

        trajectory_data = request.trajectory

        joint_names = trajectory_data.joint_names
        logger.info(trajectory_data.joint_names)
                
        # Overwrite joint names from message to match ones defined by MORSE        
        #for i in range( len(joint_names) ):
        #    joint_names[i] = joint_names[i].replace("kuka_joint", "kuka")
        #logger.info(joint_names)
        
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

    #@interruptible
    #@adapters.ros_action(type=FollowJointTrajectoryAction, name="follow_joint_trajectory")
    #def _hacked_follow_joint_trajectory(self, request):

        executor = TrajectoryExecutor(self.overlaid_object)
        executor.prepare_trajectory(request.trajectory)
        #executor.go()

        self.overlaid_object._my_behavior.set_trajectory(
                self.chain_callback(self._hacked_follow_joint_trajectory_result), executor._trajectory)

    def _hacked_follow_joint_trajectory_result(self, result):
        return result