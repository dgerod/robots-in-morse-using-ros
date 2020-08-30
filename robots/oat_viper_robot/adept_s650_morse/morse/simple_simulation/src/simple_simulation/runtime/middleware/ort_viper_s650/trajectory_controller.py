import logging
from collections import OrderedDict
import types
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from morse.core.services import interruptible
from morse.core.exceptions import MorseServiceError
from morse.core.morse_time import time_isafter
from morse.core import status
from morse_helpers import adapters


logger = logging.getLogger("morse." + __name__)


def _stamp_to_secs(stamp):
    return stamp.secs + stamp.nsecs / 1e9


def execute_trajectory(self):
    t = self.robot_parent.gettime()
    trajectory = self._active_trajectory

    try:
        if time_isafter(trajectory["starttime"], t):
            return

        if time_isafter(t, trajectory["starttime"] + trajectory["points"][-1]["time_from_start"]):
            # trajectory execution is over!
            self._active_trajectory = None
            # TODO: check here the final pose match the last point pose
            self.completed(status.SUCCESS, None)

        for p in trajectory["points"]:
            end = trajectory["starttime"] + p["time_from_start"]
            if "started" in p and time_isafter(end, t):
                # currently going to this waypoint: fine!
                break

            elif "started" not in p and time_isafter(end, t):
                # start the new waypoint
                allocated_time = end - t

                # assert (allocated_time > 0)
                if allocated_time <= 0:
                    self._active_trajectory = None
                    return

                target = OrderedDict(zip(self.local_data.keys(), p["positions"]))

                for joint in target.keys():
                    # compute the distance based on actual current joint pose
                    dist = target[joint] - self._get_joint_value(joint)
                    self.joint_speed[joint] = dist / allocated_time

                self.local_data = target

                p["started"] = True
                break

            elif "started" not in p and time_isafter(t, end):
                logger.warning("Skipped a waypoint on armature <%s>. Wrong 'time_from_start'?" % self.name())

    except KeyError as ex:
        self._active_trajectory = None
        self.completed(status.FAILED, "Error: invalid trajectory: key %s was expected." % ex)


class TrajectoryController(adapters.ROSController):

    def __init__(self, overlaid_object, namespace=None):

        super().__init__(overlaid_object)
        joints = list(overlaid_object.local_data.keys())

        self.namespace = namespace
        name = adapters.morse_to_ros_namespace(self.name())
        rospy.set_param(name + "/joint_names", joints)

        overlaid_object._exec_traj = types.MethodType(execute_trajectory, overlaid_object)

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    @interruptible
    @adapters.ros_action(type=FollowJointTrajectoryAction, name="follow_joint_trajectory")
    def _follow_joint_trajectory(self, request):

        trajectory_data = request.trajectory

        joint_names = trajectory_data.joint_names
        logger.info(trajectory_data.joint_names)

        # Read positions from message
        target_joints = self.overlaid_object.local_data.keys()
        logger.info(target_joints)

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
            point["time_from_start"] = _stamp_to_secs(p.time_from_start)
            points.append(point)

        trajectory = {"starttime": _stamp_to_secs(trajectory_data.header.stamp),
                      "points": points}
        logger.info(trajectory)

        self.overlaid_object.trajectory(
            self.chain_callback(self._follow_joint_trajectory_result), trajectory)

    def _follow_joint_trajectory_result(self, result):
        return result
