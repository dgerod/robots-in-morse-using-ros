import logging; logger = logging.getLogger("morse."+ __name__)

from morse_helpers import adapters
from sensor_msgs.msg import JointState


class ArmStatePublisherForKLWR(adapters.ROSPublisher):

    ros_class = JointState
    NUM_JOINTS = 7

    def default(self, ci='unused'):

        message = JointState()
        message.header = self.get_ros_header()
        
        message.name = [''] * self.NUM_JOINTS
        message.position = [0] * self.NUM_JOINTS
        message.velocity = [0] * self.NUM_JOINTS
        message.effort = [0] * self.NUM_JOINTS
        
        # Define name used to export joints
        base_name = "kuka_joint_"
        
        for i in range(7):
            message.name[i] = base_name + ("%d" % (i+1) )
            message.position[i] = self.data[ "kuka_" + ("%d" % (i+1) ) ]

        self.publish(message)
