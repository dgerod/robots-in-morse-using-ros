import logging
from sensor_msgs.msg import JointState
from morse_helpers import adapters


logger = logging.getLogger("morse." + __name__)


class RobotStatePublisher(adapters.ROSPublisher):

    _NUM_JOINTS = 6
    _ROS_MESSAGE_JOINT_BASE_NAME = "joint_"
    _LOCAL_FIELD_JOINT_BASE_NAME = "joint_"

    ros_class = JointState

    def default(self, ci='unused'):

        self._publish_topic()

    def _publish_topic(self):

        message = JointState()
        message.header = self.get_ros_header()

        message.name = [''] * self._NUM_JOINTS
        message.position = [0] * self._NUM_JOINTS
        message.velocity = [0] * self._NUM_JOINTS
        message.effort = [0] * self._NUM_JOINTS

        for i in range(self._NUM_JOINTS):
            message.name[i] = self._ROS_MESSAGE_JOINT_BASE_NAME + ("%d" % (i + 1))
            message.position[i] = self.data[self._LOCAL_FIELD_JOINT_BASE_NAME + ("%d" % (i + 1))]

        self.publish(message)
