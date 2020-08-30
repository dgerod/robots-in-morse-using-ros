import logging; logger = logging.getLogger("morse." + __name__)

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Vector3, Quaternion
from morse.middleware.ros import mathutils
from morse_helpers import adapters


def _get_orientation(data):
    """ Get the orientation from the local_data
    and return a ROS geometry_msgs.Quaternion
    """

    quaternion_msg = Quaternion()

    if 'orientation' in data:
        q = data['orientation']
    else:
        euler = mathutils.Euler((data['roll'],
                                 data['pitch'],
                                 data['yaw']))
        q = euler.to_quaternion()

    quaternion_msg.x = q.x
    quaternion_msg.y = q.y
    quaternion_msg.z = q.z
    quaternion_msg.w = q.w

    return quaternion_msg


def _get_position(data):
    """ Get the position from the local_data
    and return a ROS geometry_msgs.Vector3
    """

    position_msg = Vector3()

    if 'position' in data:
        position_msg.x = data['position'][0]
        position_msg.y = data['position'][1]
        position_msg.z = data['position'][2]
    else:
        position_msg.x = data['x']
        position_msg.y = data['y']
        position_msg.z = data['z']

    return position_msg


def _get_pose(data):
    """ Get the pose from the local_data
    and return a ROS geometry_msgs.Pose
    """

    pose = Pose()
    pose.position = _get_position(data)
    pose.orientation = _get_orientation(data)

    return pose


class PosePublisher(adapters.ROSPublisher):
    """ Publish the position and orientation of the robot as
    ROS geometry_msgs.Pose message.
    """
    ros_class = Pose
    default_frame_id = '/map'

    def default(self, ci='unused'):
        if 'valid' not in self.data or self.data['valid']:
            pose = _get_pose(self.data)
            self.publish(pose)


class PoseStampedPublisher(adapters.ROSPublisher):
    """ Publish the position and orientation of the robot
    as ROS geometry_msgs.PoseStamped message.
    """
    ros_class = PoseStamped
    default_frame_id = '/map'

    def default(self, ci='unused'):
        if 'valid' not in self.data or self.data['valid']:
            pose = PoseStamped()
            pose.header = self.get_ros_header()
            pose.pose = _get_pose(self.data)
            self.publish(pose)
