import logging; logger = logging.getLogger(__name__)

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from morse_comms.srv import PlaceJoints, PlaceJointsRequest


class PlaceJointsServiceClient:

    def __init__(self, namespace):
        self._namespace = namespace

    def namespace(self):
        return self._namespace

    def execute_trajectory(self, goal):

        joint_names, trajectory = self._prepare_trajectory(goal)

        for p in trajectory["points"]:
            self._call_service(joint_names, p["positions"])
        return True

    def move_to(self, joints):
        self._call_service(joints)

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def _prepare_trajectory(self, goal):

        trajectory_data = goal.trajectory
        joint_names = trajectory_data.joint_names
        logger.info(joint_names)

        points = []
        for p in trajectory_data.points:
            point = {}

            pos = dict(zip(joint_names, p.positions))
            point["positions"] = [pos[j] for j in joint_names if j in pos]

            vel = dict(zip(joint_names, p.velocities))
            point["velocities"] = [vel[j] for j in joint_names if j in vel]

            acc = dict(zip(joint_names, p.accelerations))
            point["accelerations"] = [acc[j] for j in joint_names if j in acc]

            point["time_from_start"] = self._stamp_to_secs(p.time_from_start)
            points.append(point)

        trajectory = {"starttime": self._stamp_to_secs(trajectory_data.header.stamp),
                       "points": points}

        return joint_names, trajectory

    def _call_service(self, joint_names, joint_values):

        request = PlaceJointsRequest()
        request.joints.name = joint_names
        request.joints.position = joint_values

        service_name = self._namespace + '/move_to_position'
        rospy.wait_for_service(service_name)

        try:
            service_function = rospy.ServiceProxy(service_name, PlaceJoints)
            response = service_function(request)
            return response.success

        except rospy.ServiceException as e:
            print("MoveJointsServiceClient::move_to_position %s" % e)


class FollowJointTrajectoryActionProvider:

    def __init__(self, service_client):

        self._sc = service_client

        action_name = self._sc.namespace() + "/follow_joint_trajectory"
        self._as = actionlib.SimpleActionServer(action_name, FollowJointTrajectoryAction, execute_cb=self._execute,
                                                auto_start=False)

    def start(self):
        self._as.start()

    def _execute(self, goal):
        self._sc.execute_trajectory(goal)
        result = FollowJointTrajectoryResult(FollowJointTrajectoryResult.SUCCESSFUL, "")
        self._as.set_succeeded(result)


class FollowJointTrajectoryForwarder:

    def __init__(self, namespace):
        self._service_client = PlaceJointsServiceClient(namespace)
        self._action_provider = FollowJointTrajectoryActionProvider(self._service_client)

    def start(self):
        self._action_provider.start()

