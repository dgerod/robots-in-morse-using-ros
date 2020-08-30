import logging; logger = logging.getLogger("morse."+ __name__)

from morse.middleware.ros import mathutils
from geometry_msgs.msg import Pose, Vector3, Quaternion
from ibp_morse.simulation_helpers import adapters


def _prepare_orientation_message(data):

    quaternion_msg = Quaternion()

    if 'orientation' in data:
        quaternion = data['orientation']
    else:
        euler = mathutils.Euler((data['roll'], data['pitch'], data['yaw']))
        quaternion = euler.to_quaternion()

    quaternion_msg.x = quaternion.x
    quaternion_msg.y = quaternion.y
    quaternion_msg.z = quaternion.z
    quaternion_msg.w = quaternion.w

    return quaternion_msg


def _prepare_position_message(data):

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


class PoseTfPublisher(adapters.ROSPublisherTF):

    _FRAME_ID = 'map'
    _CHILD_FRAME_ID = 'footprint_link'

    def initialize(self):

        self.frame_id = self.kwargs.get("frame_id", self._FRAME_ID)
        self.child_frame_id = self.kwargs.get("child_frame_id", self._CHILD_FRAME_ID)

        super().initialize()

        logger.info("Initialized the ROS TF publisher with frame_id '%s' " + \
                    "and child_frame_id '%s'", self.frame_id, self.child_frame_id)

    def default(self, ci='unused'):

        self._publish_tf_frame()

    def _publish_tf_frame(self):

        if 'valid' not in self.data or self.data['valid']:
            header = self.get_ros_header()
            self.sendTransform(_prepare_position_message(self.data),
                               _prepare_orientation_message(self.data),
                               header.stamp,
                               self.child_frame_id,
                               header.frame_id)
