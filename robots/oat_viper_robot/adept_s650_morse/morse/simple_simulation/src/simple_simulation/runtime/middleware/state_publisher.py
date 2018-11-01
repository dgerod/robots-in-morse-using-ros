import logging; logger = logging.getLogger("morse."+ __name__)

from sensor_msgs.msg import JointState
from morse_helpers import adapters


class StatePublisherForOatViper(adapters.ROSPublisher):

    ros_class = JointState
    NUM_JOINTS = 6

    def default(self, ci='unused'):

        message = JointState()
        message.header = self.get_ros_header()      
        
        message.name = [''] * self.NUM_JOINTS
        message.position = [0] * self.NUM_JOINTS
        message.velocity = [0] * self.NUM_JOINTS
        message.effort = [0] * self.NUM_JOINTS
        
        # Define name used to export joints
        base_name = "joint_"
        
        for i in range(self.NUM_JOINTS):
            message.name[i] = base_name + ("%d" % (i+1) )
            message.position[i] = self.data[ "joint_" + ("%d" % (i+1) ) ]

        self.publish(message)
